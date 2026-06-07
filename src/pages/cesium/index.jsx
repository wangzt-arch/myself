import { useCallback, useEffect, useRef, useState } from "react";
import * as Cesium from "cesium";
import "cesium/Build/Cesium/Widgets/widgets.css";
import Header from "../../components/Header";
import CityPopup from "./components/CityPopup";
import EffectLibraryPanel from "./components/EffectLibraryPanel";
import { CITIES_DATA } from "./cityData";
import { CESIUM_TOKEN, INITIAL_CAMERA, STATS, VIEWER_OPTIONS } from "./constants";
import { DRONE_ROUTE_OPTIONS, createDronePatrol, updateDroneFollowCamera } from "./drone";
import { createWebGLEffect, removeWebGLEffect } from "./effects";
import EmberSphereEffect from './effect/EmberSphereEffect'

import {
  INITIAL_LAYERS,
  createCoverageAreas,
  createFlightLines,
  createSatelliteTrack,
} from "./layers";
import "./index.scss";

Cesium.Ion.defaultAccessToken = CESIUM_TOKEN;

// 鼠标移动时实时读取经纬度并更新底部坐标栏。
function createMouseMoveHandler(viewer, updateCoordinates) {
  const handler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);
  handler.setInputAction((movement) => {
    const cartesian = viewer.camera.pickEllipsoid(movement.endPosition, viewer.scene.globe.ellipsoid);
    if (!cartesian) return;

    const cartographic = Cesium.Cartographic.fromCartesian(cartesian);
    updateCoordinates({
      lon: Cesium.Math.toDegrees(cartographic.longitude).toFixed(4),
      lat: Cesium.Math.toDegrees(cartographic.latitude).toFixed(4),
      height: Math.round(cartographic.height),
    });
  }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);

  return handler;
}

// 点击地图时优先处理 WebGL 特效落点，否则继续处理城市信息弹窗。
function createCityClickHandler(
  viewer,
  updateSelectedCity,
  updatePopupPosition,
  getActiveEffectType,
  addEffect
) {
  const handler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);
  handler.setInputAction((click) => {
    const activeEffectType = getActiveEffectType();
    if (activeEffectType) {
      const effectPosition = getMapClickPosition(viewer, click.position);
      if (effectPosition) {
        addEffect(activeEffectType, effectPosition);
      }
      updateSelectedCity(null);
      return;
    }

    const picked = viewer.scene.pick(click.position);
    if (!Cesium.defined(picked) || !Cesium.defined(picked.id)) {
      updateSelectedCity(null);
      return;
    }

    const cityName = getPickedCityName(picked.id);
    const cityData = CITIES_DATA.find((city) => city.name === cityName);
    if (!cityData) return;

    updateSelectedCity(cityData);
    updatePopupPosition(getPopupPosition(viewer, click.position));
  }, Cesium.ScreenSpaceEventType.LEFT_CLICK);

  return handler;
}

// 从屏幕点击位置换算地球表面的 Cartesian 坐标，供特效落点使用。
function getMapClickPosition(viewer, screenPosition) {
  const ray = viewer.camera.getPickRay(screenPosition);
  const globePosition = ray ? viewer.scene.globe.pick(ray, viewer.scene) : null;
  if (Cesium.defined(globePosition)) return globePosition;

  return viewer.camera.pickEllipsoid(screenPosition, viewer.scene.globe.ellipsoid);
}

// 从点击到的 Cesium 实体中读取城市名称，兼容普通 Entity 和 GeoJSON 属性。
function getPickedCityName(entity) {
  if (entity.name) return entity.name;

  try {
    return entity.properties?.name?.getValue?.();
  } catch (error) {
    return "";
  }
}

// 根据点击点计算弹窗位置，并保证弹窗不会超出视口。
function getPopupPosition(viewer, clickPosition) {
  const rect = viewer.scene.canvas.getBoundingClientRect();
  const pageX = rect.left + clickPosition.x;
  const pageY = rect.top + clickPosition.y;
  const popupWidth = 300;
  const popupHeight = 320;
  let x = pageX + 15;
  let y = pageY - popupHeight / 2;

  if (x + popupWidth > window.innerWidth - 10) x = pageX - popupWidth - 15;
  if (x < 10) x = 10;
  if (y < 10) y = 10;
  if (y + popupHeight > window.innerHeight - 10) y = window.innerHeight - popupHeight - 10;

  return { x, y };
}

// 加载 Cesium Ion 卫星影像底图。
async function loadIonImagery(viewerRef) {
  try {
    const viewer = viewerRef.current;
    if (!viewer) return;

    const provider = await Cesium.IonImageryProvider.fromAssetId(2);
    if (!viewerRef.current || viewerRef.current !== viewer || viewer.isDestroyed?.()) return;
    viewer.imageryLayers.removeAll();
    viewer.imageryLayers.addImageryProvider(provider);
  } catch (error) {
    console.error("Failed to load Ion imagery:", error);
  }
}

// 加载中国行政区划 GeoJSON，并为省份中心点创建标签。
async function loadChinaBoundary(viewerRef, provinceLabelEntitiesRef, updateLoading, applyProvinceVisibility) {
  try {
    await new Promise((resolve) => setTimeout(resolve, 2000));
    const viewer = viewerRef.current;
    if (!viewer) return;

    // 防止 React StrictMode 重复调用
    if (provinceLabelEntitiesRef.current.length > 0) return;

    const res = await fetch("/myself/china.json");
    const geojson = await res.json();
    const dataSource = new Cesium.GeoJsonDataSource("china");
    await dataSource.load(geojson, {
      stroke: Cesium.Color.fromCssColorString("#00f5ff"),
      fill: Cesium.Color.TRANSPARENT,
      strokeWidth: 2,
    });
    await viewer.dataSources.add(dataSource);

    provinceLabelEntitiesRef.current = createProvinceLabels(viewer, geojson);
    // 省份标签加载完成后，应用初始显隐状态
    if (applyProvinceVisibility) {
      applyProvinceVisibility();
    }
    updateLoading(false);
  } catch (error) {
    console.error("Failed to load China boundary:", error);
    updateLoading(false);
  }
}

// 根据省份 GeoJSON 的中心点创建省份名称标签。
function createProvinceLabels(viewer, geojson) {
  return (geojson.features || [])
    .map((feature) => {
      const [lon, lat] = feature.properties?.center || [];
      if (!feature.properties?.name || lon == null || lat == null) return null;

      const labelEntity = viewer.entities.add({
        name: feature.properties.name,
        position: Cesium.Cartesian3.fromDegrees(lon, lat),
        label: {
          text: feature.properties.name,
          font: "bold 14px sans-serif",
          fillColor: Cesium.Color.fromCssColorString("#00f5ff"),
          outlineColor: Cesium.Color.BLACK,
          outlineWidth: 2,
          style: Cesium.LabelStyle.FILL_AND_OUTLINE,
          verticalOrigin: Cesium.VerticalOrigin.CENTER,
          horizontalOrigin: Cesium.HorizontalOrigin.CENTER,
          heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
          scaleByDistance: new Cesium.NearFarScalar(1000000, 1.0, 50000000, 0.5),
        },
      });
      labelEntity.show = INITIAL_LAYERS.provinceLabels;
      return labelEntity;
    })
    .filter(Boolean);
}

// 设置页面初始相机视角。
function flyToInitialCamera(viewer) {
  const cam = INITIAL_CAMERA;
  viewer.camera.flyTo({
    destination: Cesium.Cartesian3.fromDegrees(cam.destination.lon, cam.destination.lat, cam.destination.height),
    orientation: {
      heading: Cesium.Math.toRadians(cam.orientation.heading),
      pitch: Cesium.Math.toRadians(cam.orientation.pitch),
      roll: cam.orientation.roll,
    },
    duration: cam.duration,
  });
}

// 添加城市点位和城市名称标签，用于点击弹窗和基础地理展示。
function createCityMarkers(viewer) {
  return CITIES_DATA.map((city) =>
    viewer.entities.add({
      id: city.name,
      name: city.name,
      position: Cesium.Cartesian3.fromDegrees(city.lon, city.lat),
      point: {
        pixelSize: city.name === "北京" ? 18 : 14,
        color: Cesium.Color.fromCssColorString(city.color),
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 2,
        heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
        scaleByDistance: new Cesium.NearFarScalar(1.0, 1.0, 50000000, 0.6),
      },
      label: {
        text: city.name,
        font: city.name === "北京" ? "bold 14px sans-serif" : "12px sans-serif",
        fillColor: Cesium.Color.WHITE,
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 2,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
        pixelOffset: new Cesium.Cartesian2(0, -12),
        heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
      },
    })
  );
}

// 图层创建后按照默认配置设置初始显示状态。
function applyInitialLayerVisibility(entityRefs) {
  entityRefs.city.current.forEach((entity) => { entity.show = INITIAL_LAYERS.cityMarkers; });
  entityRefs.satellite.current.forEach((entity) => { entity.show = INITIAL_LAYERS.satellite; });
  entityRefs.flightLine.current.forEach((entity) => { entity.show = INITIAL_LAYERS.flightLines; });
  entityRefs.coverage.current.forEach((entity) => { entity.show = INITIAL_LAYERS.coverage; });
}

// 从卫星图层集合中找到真正用于相机跟随的卫星主体实体。
function getSatelliteTarget(satelliteItems) {
  return satelliteItems.find((item) => item?.name === "LEO-01 Satellite Body" && item.position);
}

// 根据卫星当前位置计算近距离观察点，让跟随操作有明确的相机反馈。
function getSatelliteCameraDestination(satellitePosition) {
  const surfaceNormal = Cesium.Ellipsoid.WGS84.geodeticSurfaceNormal(
    satellitePosition,
    new Cesium.Cartesian3()
  );
  let rightDirection = Cesium.Cartesian3.cross(
    surfaceNormal,
    Cesium.Cartesian3.UNIT_Z,
    new Cesium.Cartesian3()
  );

  if (Cesium.Cartesian3.magnitude(rightDirection) < Cesium.Math.EPSILON6) {
    rightDirection = Cesium.Cartesian3.UNIT_X;
  }

  const rightOffset = Cesium.Cartesian3.multiplyByScalar(
    Cesium.Cartesian3.normalize(rightDirection, new Cesium.Cartesian3()),
    180000,
    new Cesium.Cartesian3()
  );
  const upOffset = Cesium.Cartesian3.multiplyByScalar(
    surfaceNormal,
    12800000,
    new Cesium.Cartesian3()
  );

  return Cesium.Cartesian3.add(
    satellitePosition,
    Cesium.Cartesian3.add(rightOffset, upOffset, new Cesium.Cartesian3()),
    new Cesium.Cartesian3()
  );
}

// 持续把相机绑定到卫星附近，形成真正随卫星移动的观察视角。
function updateSatelliteFollowCamera(viewer, satelliteItems) {
  const satellite = getSatelliteTarget(satelliteItems);
  const satellitePosition = satellite?.position?.getValue(viewer.clock.currentTime);
  if (!satellitePosition) return;

  viewer.camera.lookAt(
    satellitePosition,
    new Cesium.HeadingPitchRange(
      0,
      Cesium.Math.toRadians(-82),
      11200000
    )
  );
}

function CesiumPage() {
  const cesiumContainerRef = useRef(null);
  const viewerRef = useRef(null);
  const cityEntitiesRef = useRef([]);
  const provinceLabelEntitiesRef = useRef([]);
  const satelliteEntitiesRef = useRef([]);
  const flightLineEntitiesRef = useRef([]);
  const coverageEntitiesRef = useRef([]);
  const webGLEffectsRef = useRef([]);
  const droneControllerRef = useRef(null);
  const activeEffectTypeRef = useRef(null);
  const isFollowingSatelliteRef = useRef(false);
  const [isLoading, setIsLoading] = useState(true);
  const [isInfoPanelOpen, setIsInfoPanelOpen] = useState(false);
  const [isToolPanelOpen, setIsToolPanelOpen] = useState(false);
  const [isEffectPanelOpen, setIsEffectPanelOpen] = useState(false);
  const [activeEffectType, setActiveEffectType] = useState(null);
  const [webGLEffects, setWebGLEffects] = useState([]);
  const [droneState, setDroneState] = useState({
    status: "ready",
    routeKey: "city",
    routeLabel: "城市巡检",
    height: 260,
    speed: "42km/h",
    progress: 0,
    visible: true,
    following: false,
  });
  const [layers, setLayers] = useState(INITIAL_LAYERS);
  const [coordinates, setCoordinates] = useState({ lon: "105.0000", lat: "35.0000", height: 8000000 });
  const [selectedCity, setSelectedCity] = useState(null);
  const [popupPosition, setPopupPosition] = useState({ x: 0, y: 0 });

  // 关闭城市信息弹窗。
  const handleClosePopup = useCallback(() => setSelectedCity(null), []);

  // 拖动城市弹窗时更新弹窗屏幕坐标。
  const handleDragPopup = useCallback((dx, dy) => {
    setPopupPosition((prev) => ({ x: prev.x + dx, y: prev.y + dy }));
  }, []);

  // 在地图点击位置添加当前选择的 WebGL 特效，添加成功后自动取消选中状态。
  const addWebGLEffect = useCallback((type, position) => {
    const viewer = viewerRef.current;
    if (!viewer || !position) return;

    const effect = createWebGLEffect(viewer, type, position, webGLEffectsRef.current.length + 1);
    webGLEffectsRef.current = [effect, ...webGLEffectsRef.current];
    setWebGLEffects(webGLEffectsRef.current);

    // 添加成功后清除当前特效选中状态
    setActiveEffectType(null);
  }, []);

  const syncDroneState = useCallback(() => {
    const controller = droneControllerRef.current;
    if (!controller) return;

    setDroneState(controller.getSnapshot());
  }, []);

  const setDroneFollowing = useCallback((following) => {
    const viewer = viewerRef.current;
    const controller = droneControllerRef.current;
    if (!viewer || !controller) return;

    controller.following = following;
    if (following) {
      isFollowingSatelliteRef.current = false;
      viewer.trackedEntity = undefined;
      viewer.clock.shouldAnimate = true;
      controller.start();
      updateDroneFollowCamera(viewer, controller);
    } else {
      viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
    }
    syncDroneState();
  }, [syncDroneState]);

  const startDroneFlight = useCallback(() => {
    const viewer = viewerRef.current;
    const controller = droneControllerRef.current;
    if (!viewer || !controller) return;

    controller.start();
    viewer.clock.shouldAnimate = true;

    // 启动时将视角飞到无人机上方俯视
    const dronePos = controller.getPosition();
    if (dronePos) {
      viewer.camera.flyTo({
        destination: Cesium.Cartesian3.fromDegrees(
          Cesium.Math.toDegrees(Cesium.Cartographic.fromCartesian(dronePos).longitude),
          Cesium.Math.toDegrees(Cesium.Cartographic.fromCartesian(dronePos).latitude),
          Cesium.Cartographic.fromCartesian(dronePos).height + 5200
        ),
        orientation: {
          heading: 0,
          pitch: Cesium.Math.toRadians(-90),
          roll: 0,
        },
        duration: 1.5,
      });
    }

    syncDroneState();
  }, [syncDroneState]);

  const pauseDroneFlight = useCallback(() => {
    const controller = droneControllerRef.current;
    if (!controller) return;

    controller.pause();
    syncDroneState();
  }, [syncDroneState]);

  const resetDroneFlight = useCallback(() => {
    const controller = droneControllerRef.current;
    if (!controller) return;

    controller.reset();
    syncDroneState();
  }, [syncDroneState]);

  const toggleDroneVisible = useCallback(() => {
    const controller = droneControllerRef.current;
    if (!controller) return;

    controller.setVisible(!controller.visible);
    syncDroneState();
  }, [syncDroneState]);

  const changeDroneRoute = useCallback((routeKey) => {
    const viewer = viewerRef.current;
    const controller = droneControllerRef.current;
    if (!viewer || !controller || controller.routeKey === routeKey) return;

    const wasFollowing = controller.following;
    const shouldKeepFlying = controller.status === "flying" || wasFollowing;
    controller.setRoute(routeKey);
    controller.following = wasFollowing;
    if (shouldKeepFlying) {
      controller.start();
      viewer.clock.shouldAnimate = true;
    }
    if (wasFollowing) {
      updateDroneFollowCamera(viewer, controller);
    }
    syncDroneState();
  }, [syncDroneState]);

  // 删除指定 WebGL 特效，并同步右侧列表。
  const deleteWebGLEffect = useCallback((id) => {
    const viewer = viewerRef.current;
    if (!viewer) return;

    const target = webGLEffectsRef.current.find((effect) => effect.id === id);
    removeWebGLEffect(viewer, target);
    webGLEffectsRef.current = webGLEffectsRef.current.filter((effect) => effect.id !== id);
    setWebGLEffects(webGLEffectsRef.current);
  }, []);

  // 清空地图上所有手动添加的 WebGL 特效。
  const clearWebGLEffects = useCallback(() => {
    const viewer = viewerRef.current;
    if (!viewer) return;

    webGLEffectsRef.current.forEach((effect) => removeWebGLEffect(viewer, effect));
    webGLEffectsRef.current = [];
    setWebGLEffects([]);
  }, []);

  // 将当前选择的 WebGL 特效添加到视图中心，作为快速演示入口。
  const addEffectAtViewCenter = useCallback(() => {
    const viewer = viewerRef.current;
    const type = activeEffectTypeRef.current;
    if (!viewer || !type) return;

    const canvas = viewer.scene.canvas;
    const center = new Cesium.Cartesian2(canvas.clientWidth / 2, canvas.clientHeight / 2);
    const effectPosition = getMapClickPosition(viewer, center);
    if (!effectPosition) return;

    addWebGLEffect(type, effectPosition);
  }, [addWebGLEffect]);

  // 切换功能面板中的图层开关状态。
  const toggleLayer = useCallback((key) => {
    setLayers((prev) => ({ ...prev, [key]: !prev[key] }));
  }, []);

  // 将相机飞回中国区域的默认视角。
  const flyToChina = useCallback(() => {
    const viewer = viewerRef.current;
    const cam = INITIAL_CAMERA;
    if (!viewer) return;

    isFollowingSatelliteRef.current = false;
    if (droneControllerRef.current) {
      droneControllerRef.current.following = false;
      syncDroneState();
    }
    viewer.trackedEntity = undefined;
    viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
    viewer.camera.flyTo({
      destination: Cesium.Cartesian3.fromDegrees(cam.destination.lon, cam.destination.lat, cam.destination.height),
      orientation: {
        heading: Cesium.Math.toRadians(cam.orientation.heading),
        pitch: Cesium.Math.toRadians(cam.orientation.pitch),
        roll: cam.orientation.roll,
      },
      duration: 1.4,
    });
  }, [syncDroneState]);

  // 开启卫星图层并让相机跟随卫星实体运动。
  const flyToSatellite = useCallback(() => {
    const viewer = viewerRef.current;
    const satellite = getSatelliteTarget(satelliteEntitiesRef.current);
    if (!viewer || !satellite) return;

    setLayers((prev) => ({ ...prev, satellite: true }));
    satelliteEntitiesRef.current.forEach((entity) => {
      entity.show = true;
    });
    viewer.clock.shouldAnimate = true;
    viewer.trackedEntity = undefined;
    viewer.camera.cancelFlight();
    isFollowingSatelliteRef.current = true;
    if (droneControllerRef.current) {
      droneControllerRef.current.following = false;
      syncDroneState();
    }

    const satellitePosition = satellite.position.getValue(viewer.clock.currentTime);
    if (!satellitePosition) {
      updateSatelliteFollowCamera(viewer, satelliteEntitiesRef.current);
      return;
    }

    const destination = getSatelliteCameraDestination(satellitePosition);
    viewer.camera.flyTo({
      destination,
      orientation: {
        direction: Cesium.Cartesian3.normalize(
          Cesium.Cartesian3.subtract(satellitePosition, destination, new Cesium.Cartesian3()),
          new Cesium.Cartesian3()
        ),
        up: Cesium.Ellipsoid.WGS84.geodeticSurfaceNormal(satellitePosition, new Cesium.Cartesian3()),
      },
      duration: 0.8,
      complete: () => {
        updateSatelliteFollowCamera(viewer, satelliteEntitiesRef.current);
      },
    });
  }, [syncDroneState]);

  // 取消相机跟随，恢复用户自由旋转缩放地球。
  const stopTracking = useCallback(() => {
    const viewer = viewerRef.current;
    if (!viewer) return;

    isFollowingSatelliteRef.current = false;
    if (droneControllerRef.current) {
      droneControllerRef.current.following = false;
      syncDroneState();
    }
    viewer.trackedEntity = undefined;
    viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
  }, [syncDroneState]);
  //添加太阳特效（固定在地球左上方）
  const addSun=(viewer)=>{
    new EmberSphereEffect(viewer, {
      longitude: -60,    // 西经60度，地球左上方
      latitude: 45,      // 北纬45度
      height: 8000000,   // 抬高高度
      radius: 1500000,   // 太阳大小
      speed: 0.3,        // 较慢的流动速度
      autoAnimate: true,
      Cesium:Cesium
    })
  }

  // 当图层开关变化时，同步控制对应 Cesium 实体的显示隐藏。
  useEffect(() => {
    cityEntitiesRef.current.forEach((entity) => {
      entity.show = layers.cityMarkers;
    });
    // 省份标签：优先使用 ref 缓存，如果为空则直接从 viewer.entities 中查找
    const viewer = viewerRef.current;
    if (viewer && provinceLabelEntitiesRef.current.length > 0) {
      provinceLabelEntitiesRef.current.forEach((entity) => {
        if (entity) entity.show = layers.provinceLabels;
      });
    } else if (viewer) {
      // ref 为空时，直接遍历 viewer.entities 设置
      const entities = viewer.entities.values;
      entities.forEach((entity) => {
        if (entity.label && entity.name && !entity.name.includes('Satellite')) {
          entity.show = layers.provinceLabels;
        }
      });
    }
    satelliteEntitiesRef.current.forEach((entity) => {
      entity.show = layers.satellite;
    });
    flightLineEntitiesRef.current.forEach((entity) => {
      entity.show = layers.flightLines;
    });
    coverageEntitiesRef.current.forEach((entity) => {
      entity.show = layers.coverage;
    });
  }, [layers]);

  useEffect(() => {
    activeEffectTypeRef.current = activeEffectType;
  }, [activeEffectType]);

  // 初始化 Cesium Viewer、地图底图、行政区划、城市点和演示图层。
  useEffect(() => {
    if (!cesiumContainerRef.current) return;
    // 防止 React StrictMode 重复初始化
    if (viewerRef.current) return;

    const viewer = new Cesium.Viewer(cesiumContainerRef.current, VIEWER_OPTIONS);
    viewerRef.current = viewer;

    const mouseMoveHandler = createMouseMoveHandler(viewer, setCoordinates);
    const clickHandler = createCityClickHandler(
      viewer,
      setSelectedCity,
      setPopupPosition,
      () => activeEffectTypeRef.current,
      addWebGLEffect
    );
    const removeFollowTick = viewer.clock.onTick.addEventListener(() => {
      if (isFollowingSatelliteRef.current) {
        updateSatelliteFollowCamera(viewer, satelliteEntitiesRef.current);
      }
      if (droneControllerRef.current?.following) {
        updateDroneFollowCamera(viewer, droneControllerRef.current);
      }
    });
    const droneStateTimer = window.setInterval(() => {
      if (droneControllerRef.current) {
        setDroneState(droneControllerRef.current.getSnapshot());
      }
    }, 600);

    loadIonImagery(viewerRef);
    loadChinaBoundary(viewerRef, provinceLabelEntitiesRef, setIsLoading, () => {
      // 省份标签加载完成后应用初始显隐状态
      provinceLabelEntitiesRef.current.forEach((entity) => {
        if (entity) entity.show = INITIAL_LAYERS.provinceLabels;
      });
    });
    flyToInitialCamera(viewer);
    cityEntitiesRef.current = createCityMarkers(viewer);
    satelliteEntitiesRef.current = createSatelliteTrack(viewer);
    flightLineEntitiesRef.current = createFlightLines(viewer);
    coverageEntitiesRef.current = createCoverageAreas(viewer);
    droneControllerRef.current = createDronePatrol(viewer, "city");
    setDroneState(droneControllerRef.current.getSnapshot());
    applyInitialLayerVisibility({
      city: cityEntitiesRef,
      satellite: satelliteEntitiesRef,
      flightLine: flightLineEntitiesRef,
      coverage: coverageEntitiesRef,
    });
    addSun(viewer)

    // 组件卸载时销毁 Cesium 事件和 Viewer，避免 WebGL 资源泄漏。
    return () => {
      isFollowingSatelliteRef.current = false;
      window.clearInterval(droneStateTimer);
      removeFollowTick();
      mouseMoveHandler.destroy();
      clickHandler.destroy();
      cityEntitiesRef.current = [];
      provinceLabelEntitiesRef.current = [];
      satelliteEntitiesRef.current = [];
      flightLineEntitiesRef.current = [];
      coverageEntitiesRef.current = [];
      droneControllerRef.current?.remove();
      droneControllerRef.current = null;
      webGLEffectsRef.current.forEach((effect) => removeWebGLEffect(viewer, effect));
      webGLEffectsRef.current = [];
      if (viewerRef.current) {
        viewerRef.current.destroy();
        viewerRef.current = null;
      }
    };
  }, [addWebGLEffect]);

  return (
    <div className="cesium-page">
      <Header />
      <main className="cesium-main">
        {isLoading && (
          <div className="loading-overlay">
            <div className="loading-spinner"></div>
            <div className="loading-text">Initializing System...</div>
          </div>
        )}

        {isInfoPanelOpen ? (
          <div className="info-panel">
            <div className="panel-header">
              <div className="panel-title">
                <div className="status-indicator"></div>
                <h2>Earth Monitor</h2>
              </div>
              <button className="panel-close" type="button" onClick={() => setIsInfoPanelOpen(false)}>
                x
              </button>
            </div>
            <div className="panel-content">
              {Object.entries(STATS).map(([key, value]) => (
                <div className="data-row" key={key}>
                  <span className="label">{key.charAt(0).toUpperCase() + key.slice(1)}</span>
                  <span className="value">{value}</span>
                </div>
              ))}
            </div>
          </div>
        ) : (
          <button className="panel-restore" type="button" onClick={() => setIsInfoPanelOpen(true)}>
            Monitor
          </button>
        )}

        <div className={isToolPanelOpen ? "feature-panel" : "feature-panel feature-panel--collapsed"}>
          <div className="feature-panel__header">
            <div>
              <span>Mission Control</span>
              <strong>功能面板</strong>
            </div>
            <button type="button" onClick={() => setIsToolPanelOpen((value) => !value)}>
              {isToolPanelOpen ? "收起" : "展开"}
            </button>
          </div>

          {isToolPanelOpen && (
            <div className="feature-panel__body">
              <section className="feature-section">
                <div className="feature-section__title">
                  <span>View</span>
                  <strong>视角控制</strong>
                </div>
              <div className="feature-panel__actions">
                <button type="button" onClick={flyToChina}>回到中国</button>
                <button type="button" onClick={flyToSatellite}>跟随卫星</button>
                <button type="button" onClick={stopTracking}>自由视角</button>
              </div>
              </section>

              <section className="feature-section">
                <div className="feature-section__title">
                  <span>Layers</span>
                  <strong>图层显示</strong>
                </div>
              <div className="feature-panel__toggles">
                {[
                  ["satellite", "卫星轨迹"],
                  ["flightLines", "城市飞线"],
                  ["coverage", "覆盖范围"],
                  ["cityMarkers", "城市标记"],
                  ["provinceLabels", "省份标签"],
                ].map(([key, label]) => (
                  <button
                    className={layers[key] ? "layer-toggle layer-toggle--active" : "layer-toggle"}
                    key={key}
                    type="button"
                    onClick={() => toggleLayer(key)}
                  >
                    <span>{label}</span>
                    <i>{layers[key] ? "ON" : "OFF"}</i>
                  </button>
                ))}
              </div>

              </section>

              <section className="feature-section drone-panel">
                <div className="drone-panel__title">
                  <div>
                    <span>Low Altitude UAV</span>
                    <strong>{droneState.routeLabel}</strong>
                  </div>
                  <i>{droneState.status === "flying" ? "FLYING" : droneState.status === "paused" ? "PAUSED" : "READY"}</i>
                </div>

                <div className="drone-routes">
                  {DRONE_ROUTE_OPTIONS.map((route) => (
                    <button
                      className={droneState.routeKey === route.key ? "drone-route drone-route--active" : "drone-route"}
                      key={route.key}
                      type="button"
                      onClick={() => changeDroneRoute(route.key)}
                    >
                      {route.label}
                    </button>
                  ))}
                </div>

                <div className="drone-stats">
                  <div>
                    <span>高度</span>
                    <strong>{droneState.height}m</strong>
                  </div>
                  <div>
                    <span>速度</span>
                    <strong>{droneState.speed}</strong>
                  </div>
                  <div>
                    <span>进度</span>
                    <strong>{droneState.progress}%</strong>
                  </div>
                </div>

                <div className="drone-progress" aria-label="drone flight progress">
                  <span style={{ width: `${droneState.progress}%` }}></span>
                </div>

                <div className="drone-actions">
                  <button type="button" onClick={startDroneFlight}>开始</button>
                  <button type="button" onClick={pauseDroneFlight}>暂停</button>
                  <button type="button" onClick={resetDroneFlight}>重置</button>
                </div>

                <div className="drone-actions drone-actions--secondary">
                  <button type="button" onClick={toggleDroneVisible}>
                    {droneState.visible ? "隐藏航线" : "显示航线"}
                  </button>
                  <button type="button" onClick={() => setDroneFollowing(!droneState.following)}>
                    {droneState.following ? "退出跟随" : "跟随无人机"}
                  </button>
                </div>
              </section>

            </div>
          )}
        </div>

        <EffectLibraryPanel
          isOpen={isEffectPanelOpen}
          activeEffectType={activeEffectType}
          webGLEffects={webGLEffects}
          onToggle={() => setIsEffectPanelOpen((value) => !value)}
          onSelectEffect={setActiveEffectType}
          onCancel={() => setActiveEffectType(null)}
          onAddAtViewCenter={addEffectAtViewCenter}
          onClearEffects={clearWebGLEffects}
          onDeleteEffect={deleteWebGLEffect}
        />

        <div className="coordinates-display">
          <span>LON: {coordinates.lon} deg</span>
          <span>LAT: {coordinates.lat} deg</span>
          <span>ALT: {coordinates.height.toLocaleString()}m</span>
        </div>

        {selectedCity && (
          <div className="city-popup-wrapper" style={{ left: popupPosition.x, top: popupPosition.y }}>
            <CityPopup city={selectedCity} onClose={handleClosePopup} onDrag={handleDragPopup} />
          </div>
        )}

        <div ref={cesiumContainerRef} className="cesium-container" />
      </main>
    </div>
  );
}

export default CesiumPage;
