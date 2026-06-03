import React, { useCallback, useEffect, useRef, useState } from "react";
import * as Cesium from "cesium";
import "cesium/Build/Cesium/Widgets/widgets.css";
import Header from "../../components/Header";
import CityPopup from "./components/CityPopup";
import { CITIES_DATA } from "./cityData";
import { CESIUM_TOKEN, INITIAL_CAMERA, STATS, VIEWER_OPTIONS } from "./constants";
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

// 点击城市点时打开城市信息弹窗，点击空白位置时关闭弹窗。
function createCityClickHandler(viewer, updateSelectedCity, updatePopupPosition) {
  const handler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);
  handler.setInputAction((click) => {
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
    if (!viewerRef.current) return;
    viewer.imageryLayers.removeAll();
    viewer.imageryLayers.addImageryProvider(provider);
  } catch (error) {
    console.error("Failed to load Ion imagery:", error);
  }
}

// 加载中国行政区划 GeoJSON，并为省份中心点创建标签。
async function loadChinaBoundary(viewerRef, provinceLabelEntitiesRef, updateLoading) {
  try {
    await new Promise((resolve) => setTimeout(resolve, 2000));
    const viewer = viewerRef.current;
    if (!viewer) return;

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
          translucencyByDistance: new Cesium.NearFarScalar(1000000, 1.0, 20000000, 0.0),
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
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
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
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
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
  const isFollowingSatelliteRef = useRef(false);
  const [isLoading, setIsLoading] = useState(true);
  const [isInfoPanelOpen, setIsInfoPanelOpen] = useState(false);
  const [isToolPanelOpen, setIsToolPanelOpen] = useState(false);
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
  }, []);

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
  }, []);

  // 取消相机跟随，恢复用户自由旋转缩放地球。
  const stopTracking = useCallback(() => {
    const viewer = viewerRef.current;
    if (!viewer) return;

    isFollowingSatelliteRef.current = false;
    viewer.trackedEntity = undefined;
    viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
  }, []);

  // 当图层开关变化时，同步控制对应 Cesium 实体的显示隐藏。
  useEffect(() => {
    cityEntitiesRef.current.forEach((entity) => {
      entity.show = layers.cityMarkers;
    });
    provinceLabelEntitiesRef.current.forEach((entity) => {
      entity.show = layers.provinceLabels;
    });
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

  // 初始化 Cesium Viewer、地图底图、行政区划、城市点和演示图层。
  useEffect(() => {
    if (!cesiumContainerRef.current) return;

    const viewer = new Cesium.Viewer(cesiumContainerRef.current, VIEWER_OPTIONS);
    viewerRef.current = viewer;

    const mouseMoveHandler = createMouseMoveHandler(viewer, setCoordinates);
    const clickHandler = createCityClickHandler(viewer, setSelectedCity, setPopupPosition);
    const removeFollowTick = viewer.clock.onTick.addEventListener(() => {
      if (isFollowingSatelliteRef.current) {
        updateSatelliteFollowCamera(viewer, satelliteEntitiesRef.current);
      }
    });

    loadIonImagery(viewerRef);
    loadChinaBoundary(viewerRef, provinceLabelEntitiesRef, setIsLoading);
    flyToInitialCamera(viewer);
    cityEntitiesRef.current = createCityMarkers(viewer);
    satelliteEntitiesRef.current = createSatelliteTrack(viewer);
    flightLineEntitiesRef.current = createFlightLines(viewer);
    coverageEntitiesRef.current = createCoverageAreas(viewer);
    applyInitialLayerVisibility({
      city: cityEntitiesRef,
      satellite: satelliteEntitiesRef,
      flightLine: flightLineEntitiesRef,
      coverage: coverageEntitiesRef,
    });

    // 组件卸载时销毁 Cesium 事件和 Viewer，避免 WebGL 资源泄漏。
    return () => {
      isFollowingSatelliteRef.current = false;
      removeFollowTick();
      mouseMoveHandler.destroy();
      clickHandler.destroy();
      cityEntitiesRef.current = [];
      provinceLabelEntitiesRef.current = [];
      satelliteEntitiesRef.current = [];
      flightLineEntitiesRef.current = [];
      coverageEntitiesRef.current = [];
      if (viewerRef.current) {
        viewerRef.current.destroy();
        viewerRef.current = null;
      }
    };
  }, []);

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
            <>
              <div className="feature-panel__actions">
                <button type="button" onClick={flyToChina}>回到中国</button>
                <button type="button" onClick={flyToSatellite}>跟随卫星</button>
                <button type="button" onClick={stopTracking}>自由视角</button>
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
            </>
          )}
        </div>

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
