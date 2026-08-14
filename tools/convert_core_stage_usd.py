#!/usr/bin/env python3
"""Isaac 版の CoRE ステージ USD を、Unity のシミュレータが読める SDF + STL へ変換する。

Unity_ROS2_Robot_Simulator は USD を読めないので、Isaac 版 (CoRE-jp-Isaac-Sim-ROS2-packages)
が `bring_up_core_stage.launch.py` で開いている `meshes/USD/core_stage.usd` の中身
(2026 フィールドの payload + 地面 + 壁 4 枚) を、こちらで扱える形式へ書き出す。

出力形式について
----------------
メッシュは **バイナリ STL** で、座標は ROS 慣習 (Z-up, メートル) のまま出す。
シミュレータの SDF ローダは拡張子で読み方を変えており、STL だけは URDF インポータと
同じローダを通って **頂点単位で ROS→Unity 変換**される。Collada の場合は
GameObject の transform に回転を当てる実装で、その後 SDF の pose 代入
(ObjectSpawner.ApplyTransform) に上書きされて 90 度ずれるため、STL を選んでいる。

色は STL に入らないので、マテリアルごとにファイルを分け、SDF 側の
`<material><diffuse>` で与える。USD のマテリアル名は Isaac が付ける
`Opaque_<R>_<G>_<B>_` 形式なので、そこから色を取り、取れないものは
UsdShade の diffuseColor を見る。

使い方
------
    pip install --target /tmp/usdlib usd-core numpy
    PYTHONPATH=/tmp/usdlib python3 tools/convert_core_stage_usd.py \
        --usd  /path/to/CoRE-jp-Isaac-Sim-ROS2-packages/colcon_ws/src/core_stage_description/meshes/USD/core_stage.usd \
        --out  colcon_ws/src/core_stage_description \
        --min-size 0.0

`--min-size` を上げると、その寸法 (バウンディングボックスの対角長 [m]) 未満の
部品を捨てる。ボルトや小ねじは三角形数の割に見た目へ効かないので、Unity 側の
負荷を落としたいときに使う。捨てた分は STDOUT に出る。
"""

import argparse
import os
import re
import struct
import sys

import numpy as np

try:
    from pxr import Usd, UsdGeom, UsdShade, Gf
except ImportError:
    sys.exit("pxr (usd-core) が要る: pip install --target /tmp/usdlib usd-core")


# Isaac が付けるマテリアル名 "Opaque_255_0_11_" から色を取る
MATERIAL_NAME_RE = re.compile(r'^Opaque_(\d+)_(\d+)_(\d+)_')


def material_color(prim):
    """マテリアルの diffuse 色を (r, g, b) 0..1 で返す。取れなければ None。"""
    binding = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if not binding:
        return None
    name = binding.GetPrim().GetName()
    m = MATERIAL_NAME_RE.match(name)
    if m:
        return tuple(int(v) / 255.0 for v in m.groups())
    # 名前から取れない場合はシェーダの diffuseColor を辿る
    shader = binding.ComputeSurfaceSource()[0]
    if shader:
        for attr in ('diffuseColor', 'diffuse_color_constant', 'base_color'):
            inp = shader.GetInput(attr)
            if inp and inp.Get() is not None:
                c = inp.Get()
                return (float(c[0]), float(c[1]), float(c[2]))
    return None


def open_stage(usd_path):
    """core_stage.usd を開き、解決できない絶対パスの payload を隣のファイルへ差し替える。

    Isaac 版の core_stage.usd は payload をコンテナ内の絶対パス
    (/isaac-sim/colcon_ws/...) で持っているため、そのままでは解決できない。
    相対パスの payload は delete されているので、セッションレイヤで貼り直す。
    """
    stage = Usd.Stage.Open(usd_path, load=Usd.Stage.LoadNone)
    stage.SetEditTarget(stage.GetSessionLayer())
    base = os.path.dirname(os.path.abspath(usd_path))
    for prim in stage.GetPseudoRoot().GetChildren():
        pass
    for prim in stage.Traverse(Usd.PrimAllPrimsPredicate):
        payloads = prim.GetMetadata('payload')
        if not payloads:
            continue
        for item in list(payloads.prependedItems) + list(payloads.explicitItems):
            target = item.assetPath
            if os.path.isabs(target) and not os.path.isfile(target):
                local = os.path.join(base, os.path.basename(target))
                if os.path.isfile(local):
                    prim.GetPayloads().ClearPayloads()
                    prim.GetPayloads().AddPayload('./' + os.path.basename(target))
                    print(f'  payload 差し替え: {target} -> ./{os.path.basename(target)}')
                else:
                    sys.exit(f'payload {target} が見つからない (隣にも無い)')
    stage.Load()
    return stage


def collect_triangles(stage, root_path, min_size):
    """root_path 配下のメッシュを、色ごとにまとめた三角形配列にする。

    返り値: {(r,g,b): (N,3,3) float32 の頂点配列}  座標はステージ座標 (Z-up, m)。
    """
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    groups = {}
    dropped = dropped_tris = kept = kept_tris = 0

    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        if not points or not counts or not indices:
            continue

        xf = np.array(cache.GetLocalToWorldTransform(prim))
        pts = np.asarray([[p[0], p[1], p[2], 1.0] for p in points], dtype=np.float64)
        world = (pts @ xf)[:, :3]

        lo, hi = world.min(axis=0), world.max(axis=0)
        size = float(np.linalg.norm(hi - lo))

        # ポリゴンを扇状に三角形化する (USD の面は凸である前提)
        idx = np.asarray(indices, dtype=np.int64)
        tris = []
        pos = 0
        for c in counts:
            face = idx[pos:pos + c]
            pos += c
            for k in range(1, c - 1):
                tris.append((face[0], face[k], face[k + 1]))
        if not tris:
            continue

        if min_size > 0.0 and size < min_size:
            dropped += 1
            dropped_tris += len(tris)
            continue
        kept += 1
        kept_tris += len(tris)

        color = material_color(prim) or (0.7, 0.7, 0.7)
        color = tuple(round(c, 4) for c in color)
        groups.setdefault(color, []).append(world[np.asarray(tris, dtype=np.int64)])

    print(f'  採用 {kept} メッシュ / {kept_tris} 三角形')
    if dropped:
        print(f'  除外 {dropped} メッシュ / {dropped_tris} 三角形 '
              f'(対角長 < {min_size} m)')
    return {c: np.concatenate(v).astype(np.float32) for c, v in groups.items()}


def write_binary_stl(path, tris):
    """(N,3,3) の三角形配列をバイナリ STL で書く。座標はそのまま (ROS Z-up, m)。"""
    n = len(tris)
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    normals = np.cross(v1 - v0, v2 - v0)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0)

    # 12 float + uint16 の並びを一括で組む
    rec = np.zeros((n, 13), dtype=np.float32)
    rec[:, 0:3] = normals
    rec[:, 3:6] = v0
    rec[:, 6:9] = v1
    rec[:, 9:12] = v2
    blob = np.zeros((n, 50), dtype=np.uint8)
    blob[:, :48] = rec[:, :12].copy().view(np.uint8).reshape(n, 48)

    with open(path, 'wb') as f:
        f.write(b'CoRE stage exported from USD'.ljust(80, b'\0'))
        f.write(struct.pack('<I', n))
        f.write(blob.tobytes())
    return n


def wall_boxes(stage):
    """core_stage.usd の Wall* (スケールした立方体) を SDF の box として拾う。"""
    walls = []
    for prim in stage.GetPrimAtPath('/World').GetChildren():
        if not prim.GetName().startswith('Wall'):
            continue
        x = UsdGeom.Xformable(prim)
        t = np.zeros(3)
        s = np.ones(3)
        q = Gf.Quatf(1, 0, 0, 0)
        for op in x.GetOrderedXformOps():
            n = op.GetOpName()
            if n.endswith('translate'):
                t = np.array(op.Get())
            elif n.endswith('scale'):
                s = np.array(op.Get())
            elif n.endswith('orient'):
                q = op.Get()
        # Isaac の Wall は 1x1x1 の Cube プリムをスケールしたもの。
        # orient は Z 軸まわりだけなので yaw に落とす。
        w = q.GetReal()
        z = q.GetImaginary()[2]
        yaw = 2.0 * np.arctan2(z, w)
        walls.append((prim.GetName(), t, s, float(yaw)))
    return walls


SDF_HEADER = """<?xml version="1.0" ?>
<!--
  {name} — tools/convert_core_stage_usd.py が生成。手で編集しないこと。

  元データ: Isaac 版 core_stage.usd (2026 フィールドの payload + 地面 + 壁 4 枚)
  メッシュは ROS 慣習 (Z-up, メートル) のバイナリ STL。シミュレータの STL ローダが
  頂点単位で Unity 座標へ変換するので、<pose> は素の ROS 座標で書けばよい。
-->
<sdf version="1.7">
  <world name="core_stage">

    <light name="sun" type="directional">
      <pose>0 0 20 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
"""

SDF_MESH_MODEL = """
    <model name="{name}">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh><uri>{uri}</uri></mesh>
          </geometry>
          <material>
            <diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

SDF_WALL_MODEL = """
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>{sx} {sy} {sz}</size></box>
          </geometry>
          <material>
            <diffuse>0.35 0.35 0.38 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

SDF_FOOTER = """
  </world>
</sdf>
"""

URDF_HEADER = """<?xml version="1.0"?>
<!--
  {name} — tools/convert_core_stage_usd.py が生成。手で編集しないこと。

  RViz でステージを出すための URDF。シミュレータが読むのは worlds/*.world の
  ほうで、こちらは可視化専用。SDF と同じ STL を色ごとの <visual> として並べる。
-->
<robot name="core_stage" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:arg name="prefix" default="" />
  <xacro:arg name="use_sim" default="false" />

"""

# URDF の <material> 定義は <robot> 直下に置く決まり。<link> の中に書くと
# パーサが受け付けない。<visual> 側からは名前で参照する。
URDF_MATERIAL = """  <material name="{stem}_material">
    <color rgba="{r} {g} {b} 1.0"/>
  </material>
"""

URDF_LINK_HEADER = """
  <link name="world"/>

  <joint name="stage_joint" type="fixed">
    <parent link="world"/>
    <child  link="stage_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <link name="stage_link">
"""

URDF_VISUAL = """    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="file://$(find core_stage_description)/meshes/STL/{stem}.stl"/>
      </geometry>
      <material name="{stem}_material"/>
    </visual>
"""

URDF_FOOTER = """    <inertial>
      <mass value="100000"/>
      <inertia ixx="25000" ixy="0" ixz="0"
               iyy="25000" iyz="0"
               izz="25000"/>
    </inertial>
  </link>

</robot>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--usd', required=True, help='Isaac 版の core_stage.usd')
    ap.add_argument('--out', required=True,
                    help='core_stage_description パッケージのディレクトリ')
    ap.add_argument('--min-size', type=float, default=0.0,
                    help='この対角長 [m] 未満のメッシュを捨てる (既定 0 = 捨てない)')
    ap.add_argument('--field-prim', default='/World/core1_2026_field',
                    help='フィールドの prim パス')
    ap.add_argument('--world-name', default='core_stage',
                    help='出力する SDF / STL のベース名')
    ap.add_argument('--max-tris-per-file', type=int, default=400000,
                    help='STL 1 ファイルあたりの三角形数の上限 (既定 40 万 = 約 20 MB)')
    args = ap.parse_args()

    mesh_dir = os.path.join(args.out, 'meshes', 'STL')
    world_dir = os.path.join(args.out, 'worlds')
    os.makedirs(mesh_dir, exist_ok=True)
    os.makedirs(world_dir, exist_ok=True)

    print(f'USD を開く: {args.usd}')
    stage = open_stage(args.usd)
    print(f'upAxis={UsdGeom.GetStageUpAxis(stage)} '
          f'metersPerUnit={UsdGeom.GetStageMetersPerUnit(stage)}')

    print(f'フィールドを収集: {args.field_prim}')
    groups = collect_triangles(stage, args.field_prim, args.min_size)
    print(f'  色グループ: {len(groups)}')

    body = ''
    visuals = ''
    materials = ''
    total_tris = total_bytes = 0
    index = 0
    for color, tris in sorted(groups.items(), key=lambda kv: -len(kv[1])):
        # 1 ファイルが大きくなりすぎないよう分割する。GitHub は 50 MB で警告、
        # 100 MB で拒否するため。シミュレータ側は STL ローダが 65535 頂点ごとに
        # サブメッシュへ割るので、ファイル分割の有無は表示にも当たり判定にも
        # 影響しない (同じ色の <model> が増えるだけ)。
        for start in range(0, len(tris), args.max_tris_per_file):
            chunk = tris[start:start + args.max_tris_per_file]
            stem = f'{args.world_name}_{index:02d}'
            index += 1
            path = os.path.join(mesh_dir, stem + '.stl')
            n = write_binary_stl(path, chunk)
            size = os.path.getsize(path)
            total_tris += n
            total_bytes += size
            body += SDF_MESH_MODEL.format(
                name=stem, uri=f'../meshes/STL/{stem}.stl',
                r=color[0], g=color[1], b=color[2])
            materials += URDF_MATERIAL.format(
                stem=stem, r=color[0], g=color[1], b=color[2])
            visuals += URDF_VISUAL.format(stem=stem)
            print(f'  {stem}.stl  {n:>8} tris  {size/1e6:7.1f} MB  rgb={color}')

    for name, t, s, yaw in wall_boxes(stage):
        body += SDF_WALL_MODEL.format(
            name=name, x=t[0], y=t[1], z=t[2], yaw=round(yaw, 6),
            sx=s[0], sy=s[1], sz=s[2])
        print(f'  wall {name}: pos={t} size={s} yaw={yaw:.3f}')

    world_path = os.path.join(world_dir, args.world_name + '.world')
    with open(world_path, 'w') as f:
        f.write(SDF_HEADER.format(name=args.world_name + '.world'))
        f.write(body)
        f.write(SDF_FOOTER)

    urdf_dir = os.path.join(args.out, 'urdf')
    os.makedirs(urdf_dir, exist_ok=True)
    urdf_path = os.path.join(urdf_dir, args.world_name + '.urdf.xacro')
    with open(urdf_path, 'w') as f:
        f.write(URDF_HEADER.format(name=args.world_name + '.urdf.xacro'))
        f.write(materials)
        f.write(URDF_LINK_HEADER)
        f.write(visuals)
        f.write(URDF_FOOTER)

    print(f'\n{world_path}')
    print(f'{urdf_path}')
    print(f'合計 {total_tris} 三角形 / {total_bytes/1e6:.1f} MB')


if __name__ == '__main__':
    main()
