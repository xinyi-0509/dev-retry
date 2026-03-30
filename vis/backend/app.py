"""
LibHGP Web 服务
后端：FastAPI
调用：hgp_py（pybind11 扩展模块）
hgp_py.cp312-win_amd64.pyd 位于 vis/ 根目录
启动方式：在 vis/backend/ 目录下执行
  uvicorn app:app --reload --port 8000
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from fastapi import FastAPI, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from pydantic import BaseModel
import hgp_py
import uuid

app = FastAPI(title="LibHGP Web Service")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

MESH_DIR = os.path.join(os.path.dirname(__file__), "mesh_cache")
os.makedirs(MESH_DIR, exist_ok=True)

INDEX_HTML = os.path.join(os.path.dirname(__file__), "web", "index.html")

@app.get("/")
async def root():
    return FileResponse(INDEX_HTML)

@app.get("/favicon.ico")
async def favicon():
    return Response(status_code=204)

# ════════════════════════════════════════════════
# 请求/响应模型
# ════════════════════════════════════════════════

class PointsRequest(BaseModel):
    points: list[list[float]]

class SmoothRequest(BaseModel):
    vertices:   list[list[float]]
    face_id_0:  list[int]
    face_id_1:  list[int]
    face_id_2:  list[int]
    iterations: int = 5

class GeodesicRequest(BaseModel):
    mesh_id: str
    source:  list[float]
    target:  list[float]

class SlicerRequest(BaseModel):
    mesh_id:      str
    plane_normal: list[float]
    plane_ds:     list[float]

class CurvatureRequest(BaseModel):
    mesh_id:      str
    input_points: list[list[float]]

class SurfaceDecompRequest(BaseModel):
    mesh_id: str

class PolygonRequest(BaseModel):
    polygon: list[list[float]]

class PolygonOffsetRequest(BaseModel):
    polygon:  list[list[float]]
    distance: float

# ════════════════════════════════════════════════
# 工具函数
# ════════════════════════════════════════════════

def mesh_response(vertices, face_id_0, face_id_1, face_id_2,
                  mesh_id: str = None):
    result = {
        "vertices": [c for v in vertices for c in v],
        "faces":    [i for t in zip(face_id_0, face_id_1, face_id_2)
                     for i in t],
    }
    if mesh_id:
        result["mesh_id"] = mesh_id
    return result

def repair_mesh(vertices, fi0, fi1, fi2):
    """
    修复网格，使其满足 CGAL 测地线所需的严格流形条件：
    1. 合并重复顶点（距离小于 EPS 的视为同一点）
    2. 删除退化面片（含重复顶点索引的面片）
    3. 删除重复面片
    4. 统一面片法向（修复半边方向冲突）
    返回修复后的 (vertices, fi0, fi1, fi2)
    """
    EPS = 1e-8

    # ── 步骤1：合并重复顶点 ───────────────────────
    # 建立旧索引 → 新索引的映射
    unique_verts  = []
    old_to_new    = {}

    for old_idx, v in enumerate(vertices):
        found = False
        for new_idx, uv in enumerate(unique_verts):
            if (abs(v[0]-uv[0]) < EPS and
                abs(v[1]-uv[1]) < EPS and
                abs(v[2]-uv[2]) < EPS):
                old_to_new[old_idx] = new_idx
                found = True
                break
        if not found:
            old_to_new[old_idx] = len(unique_verts)
            unique_verts.append(v)

    # 用新索引重建面片
    new_fi0, new_fi1, new_fi2 = [], [], []
    for a, b, c in zip(fi0, fi1, fi2):
        na, nb, nc = old_to_new[a], old_to_new[b], old_to_new[c]

        # ── 步骤2：删除退化面片（三个顶点有重复）──
        if na == nb or nb == nc or na == nc:
            continue

        new_fi0.append(na)
        new_fi1.append(nb)
        new_fi2.append(nc)

    # ── 步骤3：删除重复面片 ───────────────────────
    seen      = set()
    dedup_fi0, dedup_fi1, dedup_fi2 = [], [], []
    for a, b, c in zip(new_fi0, new_fi1, new_fi2):
        key = tuple(sorted([a, b, c]))   # 忽略顶点顺序去重
        if key not in seen:
            seen.add(key)
            dedup_fi0.append(a)
            dedup_fi1.append(b)
            dedup_fi2.append(c)

    # ── 步骤4：统一面片法向（修复半边方向冲突）──
    # 建立半边字典：(v_from, v_to) → face_idx
    # 若同一有向半边被两个面片共享，则翻转后面那个面片
    half_edge_map = {}
    fixed_fi0, fixed_fi1, fixed_fi2 = [], [], []

    for idx, (a, b, c) in enumerate(
            zip(dedup_fi0, dedup_fi1, dedup_fi2)):
        edges = [(a, b), (b, c), (c, a)]
        conflict = False
        for e in edges:
            if e in half_edge_map:
                conflict = True
                break

        if conflict:
            # 翻转面片顶点顺序（a,b,c → a,c,b）
            a, b, c = a, c, b
            edges = [(a, b), (b, c), (c, a)]

        # 再次检查翻转后是否仍有冲突，有则跳过该面片
        skip = False
        for e in edges:
            if e in half_edge_map:
                skip = True
                break

        if not skip:
            for e in edges:
                half_edge_map[e] = idx
            fixed_fi0.append(a)
            fixed_fi1.append(b)
            fixed_fi2.append(c)

    print(f"[repair_mesh] "
          f"原始：{len(vertices)} 顶点 {len(fi0)} 面片 → "
          f"修复后：{len(unique_verts)} 顶点 "
          f"{len(fixed_fi0)} 面片")

    return unique_verts, fixed_fi0, fixed_fi1, fixed_fi2


def save_mesh(vertices, fi0, fi1, fi2,
              repair: bool = False) -> str:
    """
    把网格保存为 OBJ，返回 mesh_id。
    repair=True 时先做流形修复（测地线接口需要）。
    """
    if repair:
        vertices, fi0, fi1, fi2 = repair_mesh(vertices, fi0, fi1, fi2)

    mesh_id  = uuid.uuid4().hex[:8]
    filepath = os.path.join(MESH_DIR, f"{mesh_id}.obj")

    with open(filepath, "w") as f:
        f.write("# LibHGP mesh cache\n")
        for v in vertices:
            f.write(f"v {v[0]} {v[1]} {v[2]}\n")
        for i in range(len(fi0)):
            f.write(f"f {fi0[i]+1} {fi1[i]+1} {fi2[i]+1}\n")

    return mesh_id


def get_mesh_path(mesh_id: str) -> str:
    path = os.path.join(MESH_DIR, f"{mesh_id}.obj")
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"mesh_id={mesh_id} 对应的缓存文件不存在，"
            "请先调用凸包或平滑接口生成网格"
        )
    return path

# ════════════════════════════════════════════════
# 接口 1：3D 凸包
# ═════════════════════════════════���══════════════

@app.post("/api/convex_hull_3d")
def convex_hull_3d(req: PointsRequest):
    hull_verts, fi0, fi1, fi2 = hgp_py.HGP_3D_Convex_Hulls_C2(req.points)
    # repair=True：凸包输出可能含重复顶点和方向冲突，需要修复
    mesh_id = save_mesh(hull_verts, fi0, fi1, fi2, repair=True)
    return mesh_response(hull_verts, fi0, fi1, fi2, mesh_id)

# ════════════════════════════════════════════════
# 接口 2：拉普拉斯平滑
# ════════════════════════════════════════════════

@app.post("/api/smooth")
def smooth(req: SmoothRequest):
    verts_out, fi0, fi1, fi2 = hgp_py.HGP_Mesh_Laplace_Smooth_C2(
        req.vertices, req.face_id_0,
        req.face_id_1, req.face_id_2,
        req.iterations
    )
    # 平滑不改变拓扑，不需要 repair
    mesh_id = save_mesh(verts_out, fi0, fi1, fi2, repair=False)
    return mesh_response(verts_out, fi0, fi1, fi2, mesh_id)

# ════════════════════════════════════════════════
# 接口 3：网格曲率
# ════════════════════════════════════════════════

@app.post("/api/curvature")
def curvature(req: CurvatureRequest):
    mesh_path = get_mesh_path(req.mesh_id)
    max_curs, min_curs, max_dirs, min_dirs = hgp_py.HGP_Curvature_Mesh(
        mesh_path, req.input_points
    )
    return {
        "max_curvatures": max_curs,
        "min_curvatures": min_curs,
        "max_directions": [[d[0], d[1], d[2]] for d in max_dirs],
        "min_directions": [[d[0], d[1], d[2]] for d in min_dirs],
    }

# ════════════════════════════════════════════════
# 接口 4：最短测地路径
# ════════════════════════════════════════════════

@app.post("/api/geodesic_path")
def geodesic_path(req: GeodesicRequest):
    mesh_path   = get_mesh_path(req.mesh_id)
    path_points = hgp_py.HGP_Shortest_Geodesic_Path_C3(
        mesh_path, req.source, req.target
    )
    # 同时计算距离数值（调用独立的 HGP_Geodesic_Distance）
    dist = hgp_py.HGP_Geodesic_Distance(
        mesh_path, req.source, req.target
    )
    return {
        "path":     [[p[0], p[1], p[2]] for p in path_points],
        "distance": dist,
    }

# ════════════════════════════════════════════════
# 接口 5：网格切片
# ════════════════════════════════════════════════

@app.post("/api/slicer")
def slicer(req: SlicerRequest):
    mesh_path          = get_mesh_path(req.mesh_id)
    offsetses, offsets = hgp_py.HGP_Slicer_Mesh(
        mesh_path, req.plane_normal, req.plane_ds
    )
    return {
        "slices": [
            {"contour": [[p[0], p[1], p[2]] for p in contour]}
            for contour in offsets
        ]
    }

# ════════════════════════════════════════════════
# 接口 6：表面分解
# ════════════════════════════════════════════════

@app.post("/api/surface_decomposition")
def surface_decomposition(req: SurfaceDecompRequest):
    mesh_path = get_mesh_path(req.mesh_id)
    face_sdf, regions_nb, face_segments = \
        hgp_py.HGP_Surface_Decomposition(mesh_path)
    return {
        "regions_count": regions_nb,
        "face_sdf":      face_sdf,
        "face_segments": face_segments,
    }

# ════════════════════════════════════════════════
# 接口 7：2D 多边形 Offset
# ════════════════════════════════════════════════

@app.post("/api/polygon_offset_2d")
def polygon_offset_2d(req: PolygonOffsetRequest):
    result_polys = hgp_py.HGP_2D_Polygon_One_Offsets(
        req.polygon, req.distance
    )
    return {"polygons": result_polys}

# ════════════════════════════════════════════════
# 接口 8：2D 凸包
# ════════════════════════════════════════════════

@app.post("/api/convex_hull_2d")
def convex_hull_2d(req: PolygonRequest):
    hull = hgp_py.HGP_2D_Convex_Hulls(req.polygon)
    return {"hull": hull}

# ════════════════════════════════════════════════
# 接口 9：2D 多边形采样
# ════════════════════════════════════════════════

@app.post("/api/polygon_sampling_2d")
def polygon_sampling_2d(req: PolygonRequest,
                         density: float = 0.05):
    pts = hgp_py.HGP_2D_Polygon_Regular_Sampling_C1(
        req.polygon, density
    )
    return {"points": pts}

# ════════════════════════════════════════════════
# 接口 10：网格边界提取
# 对应 HGP_3D_Triangle_Mesh_Boundary_C3
# 输出：边界轮廓线段组（前端高亮渲染）
# ════════════════════════════════════════════════

@app.post("/api/mesh_boundary")
def mesh_boundary(req: CurvatureRequest):
    """
    输入：mesh_id（网格缓存）
    输出：边界轮廓线列表，每条线是有序点序列
    """
    mesh_path = get_mesh_path(req.mesh_id)
    # 读取缓存网格顶点和面片
    verts, fi0, fi1, fi2 = _load_mesh_from_obj(mesh_path)
    boundaries = hgp_py.HGP_3D_Triangle_Mesh_Boundary_C3(
        verts, fi0, fi1, fi2
    )
    return {
        "boundaries": [
            [[p[0], p[1], p[2]] for p in loop]
            for loop in boundaries
        ]
    }


# ════════════════════════════════════════════════
# 接口 11：顶点法向量
# 对应 HGP_3D_Mesh_Normal_C2
# 输出：每个顶点的法向量（前端渲染箭头）
# ════════════════════════════════════════════════

@app.post("/api/mesh_normals")
def mesh_normals(req: CurvatureRequest):
    """
    输入：mesh_id
    输出：每个顶点的法向量 [[nx,ny,nz], ...]
    """
    mesh_path = get_mesh_path(req.mesh_id)
    verts, fi0, fi1, fi2 = _load_mesh_from_obj(mesh_path)
    normals = hgp_py.HGP_3D_Mesh_Normal_C2(verts, fi0, fi1, fi2)
    return {
        "vertices": [[v[0], v[1], v[2]] for v in verts],
        "normals":  [[n[0], n[1], n[2]] for n in normals],
    }


# ════════════════════════════════════════════════
# 接口 12：包围盒
# 对应 HGP_3D_Mesh_Boundingbox_C2
# 输出：min_corner / max_corner（前端渲染线框盒）
# ════════════════════════════════════════════════

@app.post("/api/mesh_bbox")
def mesh_bbox(req: CurvatureRequest):
    """
    输入：mesh_id
    输出：包围盒的最小角点和最大角点
    """
    mesh_path = get_mesh_path(req.mesh_id)
    verts, fi0, fi1, fi2 = _load_mesh_from_obj(mesh_path)
    min_c, max_c = hgp_py.HGP_3D_Mesh_Boundingbox_C2(verts)
    return {
        "min_corner": [min_c[0], min_c[1], min_c[2]],
        "max_corner": [max_c[0], max_c[1], max_c[2]],
    }


# ════════════════════════════════════════════════
# 接口 13：网格表面采样
# 对应 HGP_3D_Mesh_Regular_Sampling_C1
# 输出：采样点云（前端渲染散点）
# ════════════════════════════════════════════════

class SamplingRequest(BaseModel):
    mesh_id: str
    density: float = 0.05   # 包围盒对角线长度的百分比

@app.post("/api/mesh_sampling")
def mesh_sampling(req: SamplingRequest):
    """
    输入：mesh_id + 采样密度
    输出：网格表面上均匀采样的点云
    """
    mesh_path = get_mesh_path(req.mesh_id)
    pts = hgp_py.HGP_3D_Mesh_Regular_Sampling_C1(
        mesh_path, req.density
    )
    return {"points": [[p[0], p[1], p[2]] for p in pts]}


# ════════════════════════════════════════════════
# 接口 14：Loop 细分一步
# 对应 HGP_Mesh_Loop_Subdivision_One_Step
# 输出：细分后的网格（面片数约为原来 4 倍）
# ════════════════════════════════════════════════

@app.post("/api/subdivide")
def subdivide(req: SmoothRequest):
    """
    输入：三角网格（顶点+面片）
    输出：Loop 细分一步后的新网格 + mesh_id
    注意：每次调用面片数 ×4，��议不超过 3 次
    """
    # Loop 细分原地修改，传拷贝进去
    verts = [list(v) for v in req.vertices]
    fi0   = list(req.face_id_0)
    fi1   = list(req.face_id_1)
    fi2   = list(req.face_id_2)
    hgp_py.HGP_Mesh_Loop_Subdivision_One_Step(verts, fi0, fi1, fi2)
    mesh_id = save_mesh(verts, fi0, fi1, fi2, repair=False)
    return mesh_response(verts, fi0, fi1, fi2, mesh_id)


# ════════════════════════════════════════════════
# 接口 15：测地线距离（数值）
# 对应 HGP_Geodesic_Distance
# 输出：两点之间的测地线距离（标量）
# ════════════════════════════════════════════════

@app.post("/api/geodesic_distance")
def geodesic_distance(req: GeodesicRequest):
    """
    输入：mesh_id + source + target
    输出：测地线距离（浮点数）
    """
    mesh_path = get_mesh_path(req.mesh_id)
    dist = hgp_py.HGP_Geodesic_Distance(
        mesh_path, req.source, req.target
    )
    return {"distance": dist}


# ════════════════════════════════════════════════
# 接口 16：2D 多边形布尔并集
# 对应 HGP_2D_Two_Polygons_Union
# 输出：并集后的多边形组
# ════════════════════════════════════════════════

class TwoPolygonRequest(BaseModel):
    polygon_0: list[list[float]]
    polygon_1: list[list[float]]

@app.post("/api/polygon_union_2d")
def polygon_union_2d(req: TwoPolygonRequest):
    """
    输入：两个 2D 多边形
    输出：布尔并集后的多边形组
    """
    area, union_polys = hgp_py.HGP_2D_Two_Polygons_Union(
    req.polygon_0, req.polygon_1
    )
    return {
        "area":     area,
        "polygons": union_polys,
    }


# ════════════════════════════════════════════════
# 接口 17：最近点到网格距离（热力图用）
# 对应 HGP_3D_Distance_Point_Mesh
# 输出：每个查询点到网格的最近距离
# ════════════════════════════════════════════════

class DistanceQueryRequest(BaseModel):
    mesh_id:      str
    query_points: list[list[float]]

@app.post("/api/point_mesh_distance")
def point_mesh_distance(req: DistanceQueryRequest):
    """
    输入：mesh_id + 查询点列表
    输出：每个点到网格的最近距离（可用于热力图着色）
    """
    mesh_path = get_mesh_path(req.mesh_id)
    distances = hgp_py.HGP_3D_Distance_Point_Mesh(
        mesh_path, req.query_points
    )
    return {"distances": distances}


# ════════════════════════════════════════════════
# 接��� 18：2D 多边形采样
# 对应 HGP_2D_Polygon_Regular_Sampling_C1
# 输出：多边形内均匀采样点
# ════════════════════════════════════════════════

class Polygon2DSamplingRequest(BaseModel):
    polygon: list[list[float]]
    density: float = 0.05

@app.post("/api/polygon_sampling_2d_regular")
def polygon_sampling_2d_regular(req: Polygon2DSamplingRequest):
    """
    输入：2D 多边形 + 采样密度
    输出：多边形内均匀采样点列表
    """
    pts = hgp_py.HGP_2D_Polygon_Regular_Sampling_C1(
        req.polygon, req.density
    )
    return {"points": pts}


# ════════════════════════════════════════════════
# 辅助：从 OBJ 文件读回顶点和面片
# （供 boundary / normals / bbox 等接口使用）
# ════════════════════════════════════════════════

def _load_mesh_from_obj(path: str):
    """读取 OBJ 文件，返回 (verts, fi0, fi1, fi2)"""
    verts, fi0, fi1, fi2 = [], [], [], []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == 'v':
                verts.append([float(parts[1]),
                               float(parts[2]),
                               float(parts[3])])
            elif parts[0] == 'f':
                # OBJ 索引从 1 开始
                fi0.append(int(parts[1]) - 1)
                fi1.append(int(parts[2]) - 1)
                fi2.append(int(parts[3]) - 1)
    return verts, fi0, fi1, fi2