import * as Cesium from "cesium";
import { CITIES_DATA } from "../cityData";
import { EFFECT_TYPES } from "../effects";
import { DRONE_ROUTE_OPTIONS } from "../drone";
import { createWebGLEffect } from "../effects";

// ========== 工具定义层
// ctx 结构（由 CesiumPage 传递的 engine 提供）：
// { viewerRef, setLayers, droneControllerRef, setFollowingSatellite, resetView, clearEffects, onEffectAdded }
// 注：所有工具通过 ctx.viewerRef.current 动态获取 viewer，避免首次渲染时值为 null

// --- 1. 飞到指定城市/位置
const flyToTool = {
  name: "flyTo",
  description: "让相机飞到指定目标位置",
  keywords: ["飞到", "飞向", "前往", "定位", "camera", "fly"],
  handler(ctx, params) {
    const viewer = ctx.viewerRef?.current;
    const { lon, lat, height = 1500000 } = params;
    if (!viewer || lon == null || lat == null) {
      return { ok: false, message: "相机或坐标不可用" };
    }
    viewer.camera.flyTo({
      destination: Cesium.Cartesian3.fromDegrees(lon, lat, height),
      orientation: { heading: 0, pitch: Cesium.Math.toRadians(-88), roll: 0 },
      duration: 2.2,
    });
    return { ok: true, message: `已飞向 (${lon.toFixed(2)}, ${lat.toFixed(2)})` };
  },
};

// --- 2. 图层显隐工具
const toggleLayerTool = {
  name: "toggleLayer",
  keywords: ["显示", "开启", "关掉", "隐藏", "layer", "卫星", "飞行线", "覆盖", "省份"],
  description: "切换卫星轨迹、飞行线、覆盖范围、城市标记、省份标签的显示状态",
  layerMap: {
    卫星: "satellite",
    satellite: "satellite",
    飞行线: "flightLines",
    flight: "flightLines",
    覆盖: "coverage",
    城市: "cityMarkers",
    城市标记: "cityMarkers",
    省份: "provinceLabels",
    province: "provinceLabels",
  },
  handler(ctx, params) {
    const { layer, visible } = params;
    if (!layer || !ctx.setLayers) {
      return { ok: false, message: "未识别到图层名" };
    }
    ctx.setLayers((prev) => ({
      ...prev,
      [layer]: visible !== null ? visible : !prev[layer],
    }));
    const action = visible !== null ? (visible ? "显示" : "隐藏") : "切换";
    return { ok: true, message: `${action}图层：${layer}` };
  },
};

// --- 3. 添加特效工具
const addEffectTool = {
  name: "addEffect",
  keywords: ["特效", "烟花", "爆炸", "火球", "火焰", "effect", "台风", "闪电", "下雨", "下雪"],
  description: "在指定位置或视图中心添加 WebGL 视觉特效",
  handler(ctx, params) {
    const viewer = ctx.viewerRef?.current;
    const { type, position } = params;
    if (!viewer || !type || !position) {
      return { ok: false, message: "缺少特效类型或位置" };
    }
    const effect = createWebGLEffect(viewer, type, position);
    // 把 AI 创建的特效加入页面的追踪列表，确保可以被清除
    if (ctx.onEffectAdded && effect) ctx.onEffectAdded(effect);
    return { ok: true, message: `已添加特效：${type}` };
  },
};

// --- 4. 无人机控制工具
const droneTool = {
  name: "drone",
  keywords: ["无人机", "drone", "飞行", "跟随", "航线", "暂停", "重置", "停止"],
  description: "控制无人机：切换航线、开始/暂停/重置飞行、跟随视角",
  handler(ctx, params) {
    const controller = ctx.droneControllerRef?.current;
    if (!controller) return { ok: false, message: "无人机未就绪" };
    const { action, routeKey, following } = params;
    if (routeKey) controller.setRoute(routeKey);
    if (action === "start") controller.start();
    if (action === "pause") controller.pause();
    if (action === "reset") controller.reset();
    if (following != null) {
      controller.following = !!following;
      if (following && ctx.viewerRef?.current) {
        ctx.viewerRef.current.clock.shouldAnimate = true;
      }
    }
    return { ok: true, message: "无人机状态已更新" };
  },
};

// --- 5. 获取当前相机位置
const getCameraTool = {
  name: "getCamera",
  keywords: ["相机", "位置", "坐标", "哪里", "camera", "position"],
  description: "返回当前视角经纬度与高度",
  handler(ctx) {
    const viewer = ctx.viewerRef?.current;
    if (!viewer) return { ok: false, message: "相机不可用" };
    const cart = Cesium.Cartographic.fromCartesian(viewer.camera.position);
    const lon = Cesium.Math.toDegrees(cart.longitude).toFixed(3);
    const lat = Cesium.Math.toDegrees(cart.latitude).toFixed(3);
    const height = Math.round(cart.height);
    return {
      ok: true,
      message: `当前位置：经度 ${lon}°，纬度 ${lat}°，高度 ${height.toLocaleString()} m`,
    };
  },
};

// --- 6. 跟随卫星 / 自由视角
const followSatelliteTool = {
  name: "followSatellite",
  keywords: ["卫星", "跟随", "自由", "satellite", "追踪"],
  description: "让相机跟随卫星轨迹旋转",
  handler(ctx, params) {
    const { following } = params;
    if (ctx.setFollowingSatellite) ctx.setFollowingSatellite(!!following);
    return {
      ok: true,
      message: following ? "已切换到卫星跟随视角" : "已恢复自由视角",
    };
  },
};

// --- 7. 重置视角回到中国
const resetViewTool = {
  name: "resetView",
  keywords: ["回到", "重置", "中国", "视角", "reset", "china"],
  description: "相机回到默认中国视角",
  handler(ctx) {
    if (ctx.resetView) {
      ctx.resetView();
      return { ok: true, message: "已回到默认视角（中国）" };
    }
    return { ok: false, message: "视角重置不可用" };
  },
};

// --- 8. 列出所有可用特效
const listEffectsTool = {
  name: "listEffects",
  keywords: ["特效清单", "有哪些特效", "list"],
  description: "返回所有可用特效列表",
  handler() {
    const names = EFFECT_TYPES.map((e) => e.label).join("、");
    return { ok: true, message: `可用特效：${names}` };
  },
};

// --- 9. 清除所有特效
const clearEffectsTool = {
  name: "clearEffects",
  keywords: ["清除特效", "清空特效", "清除", "clear"],
  description: "移除地图上所有手动添加的特效",
  handler(ctx) {
    if (ctx.clearEffects) {
      ctx.clearEffects();
      return { ok: true, message: "已清除所有特效" };
    }
    return { ok: false, message: "清除操作不可用" };
  },
};

// --- 10. 帮助指令
const helpTool = {
  name: "help",
  keywords: ["帮助", "help", "怎么用", "指令"],
  description: "返回可用指令示例",
  handler() {
    return {
      ok: true,
      message:
        "你可以这样说：\n1. 飞到北京 / 飞到上海\n2. 显示卫星轨迹\n3. 在视图中心放一个烟花\n4. 让无人机沿河道飞\n5. 现在相机在哪里？\n6. 跟随卫星 / 自由视角\n7. 清除所有特效\n8. 有哪些特效",
    };
  },
};

export const TOOLS = [
  flyToTool,
  toggleLayerTool,
  addEffectTool,
  droneTool,
  getCameraTool,
  followSatelliteTool,
  resetViewTool,
  helpTool,
  listEffectsTool,
  clearEffectsTool,
];

export { CITIES_DATA, EFFECT_TYPES, DRONE_ROUTE_OPTIONS };
