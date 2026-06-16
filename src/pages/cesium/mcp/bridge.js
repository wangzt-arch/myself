import * as Cesium from "cesium";
import { TOOLS, CITIES_DATA, EFFECT_TYPES, DRONE_ROUTE_OPTIONS } from "./tools";
import { createWebGLEffect } from "../effects";

// ========== 简易 MCP 协议桥接引擎
// 输入自然语言 → 解析参数 → 调度工具 → 返回结果
// ctx 结构：{ viewerRef, setLayers, droneControllerRef, setFollowingSatellite, resetView, clearEffects, onEffectAdded }

// --- 解析目标城市（长名称优先，避免"三亚"匹配时漏了"三亚湾"）
function extractCity(text) {
  const lower = text.toLowerCase();
  const sorted = [...CITIES_DATA].sort((a, b) => b.name.length - a.name.length);
  return sorted.find((c) => lower.includes(c.name.toLowerCase())) || null;
}

// --- 是否为纯城市指令（输入就是城市名或短城市描述，不带其他功能关键词）
function isPureCityQuery(text, city) {
  if (!city) return false;
  const cleaned = text.replace(/[，,。.!！\s]/g, "").trim();
  if (cleaned === city.name) return true;
  const flyKeywords = /(飞到|飞向|前往|定位|定位到|跳转|跳转到|飞行到|飞去|去|到|看一下|看看|查看|来|到达|camera|fly|goto)/i;
  if (flyKeywords.test(text)) return true;
  return false;
}

// --- 解析经纬度（例如输入"116.4 39.9"）
function extractLonLat(text) {
  const matches = text.match(/-?\d+(\.\d+)?/g);
  if (matches && matches.length >= 2) {
    const lon = parseFloat(matches[0]);
    const lat = parseFloat(matches[1]);
    return { lon, lat };
  }
  return null;
}

// --- 解析特效类型
function extractEffect(text) {
  const lower = text.toLowerCase();
  for (const effect of EFFECT_TYPES) {
    if (lower.includes(effect.label)) return effect;
  }
  if (/烟花|firework/i.test(text)) return EFFECT_TYPES.find((e) => e.key === "firework") || null;
  if (/爆炸/i.test(text)) return EFFECT_TYPES.find((e) => e.key === "explosion") || null;
  if (/火球|火焰/i.test(text)) return EFFECT_TYPES.find((e) => e.key === "flame") || null;
  if (/下雨|雨/i.test(text)) return EFFECT_TYPES.find((e) => e.key === "areaRain") || null;
  if (/下雪|雪/i.test(text)) return EFFECT_TYPES.find((e) => e.key === "areaSnow") || null;
  if (/台风/i.test(text)) return EFFECT_TYPES.find((e) => e.key === "circleTyphoon") || null;
  if (/闪电/i.test(text)) return EFFECT_TYPES.find((e) => e.key === "volumeAreaLightning") || null;
  return null;
}

// --- 解析无人机航线
function extractDroneRoute(text) {
  const lower = text.toLowerCase();
  return (
    DRONE_ROUTE_OPTIONS.find(
      (route) =>
        lower.includes(route.key) ||
        lower.includes(route.label) ||
        text.includes(route.label)
    ) || null
  );
}

// --- 图层关键词解析
function extractLayer(text) {
  const map = {
    卫星: "satellite",
    飞行线: "flightLines",
    覆盖: "coverage",
    城市标记: "cityMarkers",
    城市: "cityMarkers",
    省份: "provinceLabels",
  };
  for (const keyword of Object.keys(map)) {
    if (text.toLowerCase().includes(keyword)) return { key: map[keyword] };
  }
  return null;
}

// --- 显隐方向
function isVisibleIntent(text) {
  if (/隐藏|关掉|关闭|不显示|停止显示/.test(text)) return false;
  if (/显示|开启|打开/.test(text)) return true;
  return null;
}

// --- 获取视图中心 Cartesian3
function getViewCenter(viewer) {
  if (!viewer) return null;
  const canvas = viewer.scene.canvas;
  const center = new Cesium.Cartesian2(canvas.clientWidth / 2, canvas.clientHeight / 2);
  const ray = viewer.camera.getPickRay(center);
  if (!ray) return null;
  return (
    viewer.scene.globe.pick(ray, viewer.scene) ||
    viewer.camera.pickEllipsoid(center, viewer.scene.globe.ellipsoid)
  );
}

// ========== 主入口（命令优先级控制）
export function dispatchCommand(input, ctx) {
  const text = (input || "").trim();
  if (!text) return { ok: false, message: "请输入指令" };

  const city = extractCity(text);

  // -- 1. 帮助指令（最优先，避免与其他关键词冲突）
  if (
    /^(帮助|help|怎么用|指令|使用说明)/i.test(text) ||
    text.toLowerCase() === "help" ||
    text === "?" ||
    text === "？"
  ) {
    return TOOLS.find((t) => t.name === "help").handler(ctx, {});
  }

  // -- 2. 城市飞行（"飞到北京" / "飞向上海" / "定位成都" / "北京" / 等）
  if (isPureCityQuery(text, city)) {
    return TOOLS.find((t) => t.name === "flyTo").handler(ctx, {
      lon: city.lon,
      lat: city.lat,
      height: 1500000,
    });
  }

  // -- 2.1 经纬度飞行（输入如 "116.4 39.9" 或 "飞行到 116.4 39.9"）
  const lonLat = extractLonLat(text);
  if (lonLat && /(飞到|定位|前往|飞行|fly|goto|坐标|经纬度)/i.test(text)) {
    return TOOLS.find((t) => t.name === "flyTo").handler(ctx, {
      lon: lonLat.lon,
      lat: lonLat.lat,
      height: 1500000,
    });
  }

  // -- 3. 卫星跟随（观察卫星）- 优先于图层切换
  if (/观察卫星|跟随卫星|追踪卫星|卫星跟随/i.test(text)) {
    return TOOLS.find((t) => t.name === "followSatellite").handler(ctx, { following: true });
  }
  if (/自由视角|自由/i.test(text)) {
    return TOOLS.find((t) => t.name === "followSatellite").handler(ctx, { following: false });
  }

  // -- 4. 显隐图层（排除"观察卫星"等组合）
  const layerMatch = extractLayer(text);
  if (layerMatch && !/观察卫星/i.test(text)) {
    return TOOLS.find((t) => t.name === "toggleLayer").handler(ctx, {
      layer: layerMatch.key,
      visible: isVisibleIntent(text),
    });
  }

  // -- 5. 添加特效（支持"在 X 城市 放一个烟花"这样的组合）
  const effect = extractEffect(text);
  if (effect) {
    let pos;
    if (/中心|视图|当前/i.test(text)) {
      pos = getViewCenter(ctx.viewerRef?.current) || Cesium.Cartesian3.fromDegrees(116, 39, 0);
    } else if (city) {
      pos = Cesium.Cartesian3.fromDegrees(city.lon, city.lat, 0);
    } else {
      pos = getViewCenter(ctx.viewerRef?.current) || Cesium.Cartesian3.fromDegrees(116, 39, 0);
    }
    return TOOLS
      .find((t) => t.name === "addEffect")
      .handler(ctx, { type: effect.key, position: pos });
  }

  // -- 6. 无人机控制
  const route = extractDroneRoute(text);
  if (/无人机|drone|飞行/i.test(text) || route) {
    const params = { action: "start", routeKey: route?.key };
    if (/暂停|stop|pause/i.test(text)) params.action = "pause";
    if (/重置|reset|重开/i.test(text)) params.action = "reset";
    if (/跟随|follow/i.test(text)) params.following = true;
    if (/停止跟随|自由视角/.test(text)) params.following = false;
    return TOOLS.find((t) => t.name === "drone").handler(ctx, params);
  }

  // -- 7. 相机位置查询
  if (/相机|当前位置|坐标|哪里|camera/i.test(text)) {
    return TOOLS.find((t) => t.name === "getCamera").handler(ctx);
  }

  // -- 8. 重置视角（单独的"卫星"不应该误触发飞行，所以这里要在飞行之后）
  if (/回到中国|重置视角|回到默认|默认视角|reset view/i.test(text)) {
    return TOOLS.find((t) => t.name === "resetView").handler(ctx);
  }

  // -- 9. 清除特效
  if (/清除特效|清空特效|删除特效|clear effect/i.test(text)) {
    return TOOLS.find((t) => t.name === "clearEffects").handler(ctx);
  }

  // -- 10. 特效列表
  if (/有哪些特效|特效列表|特效有什么|list effect/i.test(text)) {
    return TOOLS.find((t) => t.name === "listEffects").handler(ctx);
  }

  return {
    ok: false,
    message:
      "我暂时没有理解你的指令。试试：\n· 飞到北京\n· 观察卫星\n· 在视图中心放一个烟花\n· 让无人机沿河道飞\n· 相机位置\n· 帮助",
  };
}

// 快捷指令清单（给 UI 按钮用）
export const QUICK_COMMANDS = [
  { label: "飞到北京", text: "飞到北京" },
  { label: "飞到上海", text: "飞到上海" },
  // { label: "飞到深圳", text: "飞到深圳" },
  // { label: "飞到成都", text: "飞到成都" },
  { label: "观察卫星", text: "观察卫星" },
  { label: "放个烟花", text: "在视图中心放一个烟花" },
  // { label: "无人机河道", text: "让无人机沿河道飞" },
  { label: "相机位置", text: "现在相机在哪里" },
  { label: "清除特效", text: "清除特效" },
  { label: "帮助", text: "帮助" },
  { label: "回到中国", text: "回到中国" },
];
