"""[Isaac 3.11] PBD 물 시스템 생성 — 두 데모가 공유하는 Isaac 쪽 조립부.

repair_demo 와 real_map_demo 에 동일하게 복붙돼 있던 입자 시스템·표면
추출·재질·스폰 코드를 합쳤다 (2026-08-06). **수치는 하나도 새로 정하지
않는다** — 입자·물 값은 사용자 동결값(PCO 8 / fluid_rest 4.8 / contact 4 /
rest 2.5)이고, 호출 쪽이 그대로 넘긴다.

여기는 Isaac 조립("무엇을 만들 것인가")만 있다. 순수 수식은 water_model,
코스 기하는 course, 채움 범위·재투입 규칙("어디에 물이 있는가")은 코스마다
달라 데모에 남는다.

pxr/omni import 는 함수 안에서 한다 — crack_inject.py 와 같은 반쪽 3.11
방식이라, 3.10 오프라인에서 모듈 import 자체는 되고 호출 시점에만 Isaac 이
필요하다.
"""


def make_water_system(stage, path, *, pco, fluid_rest, contact, rest,
                      iso_passes, aniso=0.0, max_velocity=5.0):
    """입자 시스템 + isosurface(+선택 anisotropy) 생성 → 시스템 경로 반환.

    iso_passes: 표면 다듬기 횟수. 0 이면 isosurface 자체를 끈다(입자 구슬).
    aniso > 0: 낙수가 구슬로 끊겨 보이는 것 개선 — 입자를 속도 방향 타원으로
      늘려 isosurface 가 물줄기로 이어붙게 한다. **렌더 전용, 물리 무관.**
      느려지면 0 으로 끌 것. util 이름이 버전에 따라 다를 수 있어 실패 시
      PhysxSchema 를 직접 얹는다(기본값).
    """
    from omni.physx.scripts import particleUtils
    from pxr import Gf, PhysxSchema, Sdf

    psys = Sdf.Path(path)
    particleUtils.add_physx_particle_system(
        stage, psys, particle_contact_offset=pco,
        fluid_rest_offset=fluid_rest, contact_offset=contact,
        rest_offset=rest, max_velocity=max_velocity,
        wind=Gf.Vec3f(0.0, 0.0, 0.0))     # 굽은 경로는 전역 wind 못 쓴다
    particleUtils.add_physx_particle_isosurface(
        stage, psys, enabled=iso_passes > 0, grid_spacing=fluid_rest * 1.5,
        surface_distance=fluid_rest * 1.6,
        grid_smoothing_radius=fluid_rest * 2.0,
        num_mesh_smoothing_passes=iso_passes,
        num_mesh_normal_smoothing_passes=iso_passes)
    if aniso > 0:
        try:
            particleUtils.add_physx_particle_anisotropy(stage, psys,
                                                        scale=aniso)
            particleUtils.add_physx_particle_smoothing(stage, psys,
                                                       strength=0.6)
            print(f"[준비] 물줄기 이어붙임 anisotropy scale={aniso} "
                  f"+ smoothing 0.6 (렌더 전용)")
        except (AttributeError, TypeError) as e:
            _pp = stage.GetPrimAtPath(psys)
            PhysxSchema.PhysxParticleAnisotropyAPI.Apply(_pp)
            PhysxSchema.PhysxParticleSmoothingAPI.Apply(_pp)
            print(f"[준비] 물줄기 이어붙임 — util 실패({e}), "
                  f"스키마 기본값으로 적용")
    return psys


def bind_water_materials(stage, psys, *, opacity=0.35,
                         vis_root="/World/WaterVisual",
                         pbd_root="/World/WaterPBD"):
    """물 표시 재질 + PBD 물성 재질을 만들어 시스템에 바인딩한다.

    🚨 물 재질과 카메라 판정은 한몸이다 (2026-08-06). 불투명 0.85 + 강한
       발광은 물속 결함(바닥 180°)을 파란 커튼처럼 가려 카메라가 구멍을 못
       봤다(실기 보고). 기본 0.35 — 물처럼 보이면서 물속 구멍이 비쳐 보이는
       최소선이다. 올리면 검출 임계를 다시 봐야 한다.
    """
    from omni.physx.scripts import particleUtils
    from pxr import Gf, Sdf, UsdShade

    _wv = UsdShade.Material.Define(stage, vis_root)
    _ws = UsdShade.Shader.Define(stage, vis_root + "/Shader")
    _ws.CreateIdAttr("UsdPreviewSurface")
    _ws.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.15, 0.45, 0.85))
    _ws.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.04, 0.10, 0.20))
    _ws.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    _ws.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.15)
    _wv.CreateSurfaceOutput().ConnectToSource(_ws.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(psys)).Bind(
        _wv, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    _wp = UsdShade.Material.Define(stage, pbd_root)
    particleUtils.AddPBDMaterialWater(_wp.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(psys)).Bind(
        _wp, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose="physics")


def spawn_water(stage, path, positions, velocities, psys, radius):
    """입자 집합 스폰 → PointInstancer 반환 (숨김 — isosurface 가 렌더).

    positions/velocities: Gf.Vec3f 리스트. 질량은 밀도 1000 에서 자동.
    """
    from omni.physx.scripts import particleUtils
    from pxr import Sdf, UsdGeom, Vt

    inst = particleUtils.add_physx_particleset_pointinstancer(
        stage, Sdf.Path(path),
        Vt.Vec3fArray(positions), Vt.Vec3fArray(velocities),
        particle_system_path=psys, self_collision=True, fluid=True,
        particle_group=0, particle_mass=0.0, density=1000.0)
    UsdGeom.Sphere(stage.GetPrimAtPath(
        str(path) + "/particlePrototype0")).GetRadiusAttr().Set(radius)
    UsdGeom.Imageable(inst).MakeInvisible()
    return UsdGeom.PointInstancer(inst)
