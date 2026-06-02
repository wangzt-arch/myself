import React, { useEffect, useRef, useState, useCallback } from "react";
import * as Cesium from "cesium";
import "cesium/Build/Cesium/Widgets/widgets.css";
import Header from "../../components/Header";
import CityPopup from "./components/CityPopup";
import { CITIES_DATA } from "./cityData";
import { CESIUM_TOKEN, VIEWER_OPTIONS, INITIAL_CAMERA, STATS } from "./constants";
import "./index.scss";

Cesium.Ion.defaultAccessToken = CESIUM_TOKEN;

function CesiumPage() {
  const cesiumContainerRef = useRef(null);
  const viewerRef = useRef(null);
  const [isLoading, setIsLoading] = useState(true);
  const [coordinates, setCoordinates] = useState({ lon: "105.0000", lat: "35.0000", height: 8000000 });
  const [selectedCity, setSelectedCity] = useState(null);
  const [popupPosition, setPopupPosition] = useState({ x: 0, y: 0 });

  const handleClosePopup = useCallback(() => setSelectedCity(null), []);
  const handleDragPopup = useCallback((dx, dy) => {
    setPopupPosition((prev) => ({ x: prev.x + dx, y: prev.y + dy }));
  }, []);

  useEffect(() => {
    if (!cesiumContainerRef.current) return;

    const viewer = new Cesium.Viewer(cesiumContainerRef.current, VIEWER_OPTIONS);
    viewerRef.current = viewer;

    // 鼠标移动 → 实时坐标
    const mouseMoveHandler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);
    mouseMoveHandler.setInputAction((movement) => {
      const cartesian = viewer.camera.pickEllipsoid(movement.endPosition, viewer.scene.globe.ellipsoid);
      if (cartesian) {
        const cartographic = Cesium.Cartographic.fromCartesian(cartesian);
        setCoordinates({
          lon: Cesium.Math.toDegrees(cartographic.longitude).toFixed(4),
          lat: Cesium.Math.toDegrees(cartographic.latitude).toFixed(4),
          height: Math.round(cartographic.height),
        });
      }
    }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);

    // 点击拾取城市
    const clickHandler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);
    clickHandler.setInputAction((click) => {
      const picked = viewer.scene.pick(click.position);
      if (Cesium.defined(picked) && Cesium.defined(picked.id)) {
        const entity = picked.id;
        let cityName = entity.name;
        if (!cityName && entity.properties) {
          try { cityName = entity.properties.name?.getValue?.(); } catch (e) { /* ignore */ }
        }
        const cityData = CITIES_DATA.find((c) => c.name === cityName);
        if (cityData) {
          const rect = viewer.scene.canvas.getBoundingClientRect();
          const pageX = rect.left + click.position.x;
          const pageY = rect.top + click.position.y;
          const pw = 300, ph = 320;
          let x = pageX + 15, y = pageY - ph / 2;
          if (x + pw > window.innerWidth - 10) x = pageX - pw - 15;
          if (x < 10) x = 10;
          if (y < 10) y = 10;
          if (y + ph > window.innerHeight - 10) y = window.innerHeight - ph - 10;
          setSelectedCity(cityData);
          setPopupPosition({ x, y });
        }
      } else {
        setSelectedCity(null);
      }
    }, Cesium.ScreenSpaceEventType.LEFT_CLICK);

    // 加载卫星影像
    (async () => {
      try {
        const v = viewerRef.current;
        if (!v) return;
        const provider = await Cesium.IonImageryProvider.fromAssetId(2);
        if (!viewerRef.current) return;
        v.imageryLayers.removeAll();
        v.imageryLayers.addImageryProvider(provider);
      } catch (err) {
        console.error("Failed to load Ion imagery:", err);
      }
    })();

    // 加载中国行政区划
    (async () => {
      try {
        await new Promise((r) => setTimeout(r, 2000));
        const v = viewerRef.current;
        if (!v) return;

        const res = await fetch("/myself/china.json");
        const geojson = await res.json();
        const dataSource = new Cesium.GeoJsonDataSource("china");
        await dataSource.load(geojson, {
          stroke: Cesium.Color.fromCssColorString("#00f5ff"),
          fill: Cesium.Color.TRANSPARENT,
          strokeWidth: 2,
        });
        await v.dataSources.add(dataSource);

        // 添加省份标签
        geojson.features?.forEach((f) => {
          const [lon, lat] = f.properties?.center || [];
          if (f.properties?.name && lon != null && lat != null) {
            v.entities.add({
              name: f.properties.name,
              position: Cesium.Cartesian3.fromDegrees(lon, lat),
              label: {
                text: f.properties.name,
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
          }
        });

        setIsLoading(false);
      } catch (err) {
        console.error("Failed to load China boundary:", err);
        setIsLoading(false);
      }
    })();

    // 初始视角
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

    // 添加城市标记
    CITIES_DATA.forEach((city) => {
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
      });
    });

    return () => {
      mouseMoveHandler.destroy();
      clickHandler.destroy();
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

        <div className="info-panel">
          <div className="panel-header">
            <div className="status-indicator"></div>
            <h2>Earth Monitor</h2>
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

        <div className="coordinates-display">
          <span>LON: {coordinates.lon}°</span>
          <span>LAT: {coordinates.lat}°</span>
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
