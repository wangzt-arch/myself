import React, { useEffect, useRef } from "react";
import * as Cesium from "cesium";
import "cesium/Build/Cesium/Widgets/widgets.css";
import Header from "../../components/Header";
import "./index.scss";

// 设置 Cesium Ion token
Cesium.Ion.defaultAccessToken =
  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiJlZDA2OTcyOC01MTU5LTQ0MWQtODExMi0zYTZjNzE5NWYwMGYiLCJpZCI6NDM4OTQyLCJpc3MiOiJodHRwczovL2FwaS5jZXNpdW0uY29tIiwiYXVkIjoidW5kZWZpbmVkX2RlZmF1bHQiLCJpYXQiOjE3ODAzMTAyMDd9.7H3KXRDhmnCOaRVGVmzRs_snRWKTWQbhEN1sQbVbQQA";

function CesiumPage() {
  const cesiumContainerRef = useRef(null);
  const viewerRef = useRef(null);

  useEffect(() => {
    if (!cesiumContainerRef.current) return;

    const viewer = new Cesium.Viewer(cesiumContainerRef.current, {
      baseLayerPicker: false,
      geocoder: false,
      homeButton: true,
      sceneModePicker: true,
      navigationHelpButton: false,
      animation: false,
      timeline: false,
      fullscreenButton: true,
      vrButton: false,
      infoBox: false,
      selectionIndicator: false,
      targetFrameRate: 60,
    });

    viewerRef.current = viewer;

    // 使用异步方式添加 Cesium Ion Bing Maps 影像
    const loadIonImagery = async () => {
      try {
        const provider = await Cesium.IonImageryProvider.fromAssetId(2);
        viewer.imageryLayers.removeAll();
        viewer.imageryLayers.addImageryProvider(provider);
      } catch (err) {
        console.error("Failed to load Ion imagery:", err);
      }
    };
    loadIonImagery();

    // 加载中国行政区划边界和名称
    const loadChinaBoundary = async () => {
      try {
        // 等待一段时间确保viewer完全初始化
        await new Promise((resolve) => setTimeout(resolve, 2000));
        
        // 加载GeoJSON数据
        const response = await fetch("/myself/china.json");
        const geojson = await response.json();
        
        // 创建数据源
        const dataSource = new Cesium.GeoJsonDataSource("china");
        await dataSource.load(geojson, {
          stroke: Cesium.Color.WHITE,
          fill: Cesium.Color.TRANSPARENT,
          strokeWidth: 2,
        });
        
        await viewer.dataSources.add(dataSource);
        
        // 为每个行政区添加名称标签
        if (geojson.features) {
          geojson.features.forEach((feature) => {
            const name = feature.properties?.name;
            const center = feature.properties?.center;
            
            if (name && center && Array.isArray(center) && center.length >= 2) {
              const [lon, lat] = center;
              
              viewer.entities.add({
                name: name,
                position: Cesium.Cartesian3.fromDegrees(lon, lat),
                label: {
                  text: name,
                  font: "bold 14px sans-serif",
                  fillColor: Cesium.Color.WHITE,
                  outlineColor: Cesium.Color.BLACK,
                  outlineWidth: 2,
                  style: Cesium.LabelStyle.FILL_AND_OUTLINE,
                  verticalOrigin: Cesium.VerticalOrigin.CENTER,
                  horizontalOrigin: Cesium.HorizontalOrigin.CENTER,
                  pixelOffset: new Cesium.Cartesian2(0, 0),
                  heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
                  scaleByDistance: new Cesium.NearFarScalar(1000000, 1.0, 50000000, 0.5),
                  translucencyByDistance: new Cesium.NearFarScalar(1000000, 1.0, 20000000, 0.0),
                },
              });
            }
          });
        }
        
        console.log("China boundary and labels loaded successfully");
      } catch (err) {
        console.error("Failed to load China boundary:", err);
      }
    };
    loadChinaBoundary();

    // 设置初始视角 - 飞到中国上空
    viewer.camera.flyTo({
      destination: Cesium.Cartesian3.fromDegrees(105, 35, 8000000),
      orientation: {
        heading: Cesium.Math.toRadians(0),
        pitch: Cesium.Math.toRadians(-90),
        roll: 0,
      },
      duration: 3,
    });

    // 添加北京标记
    viewer.entities.add({
      name: "北京",
      position: Cesium.Cartesian3.fromDegrees(116.4074, 39.9042),
      point: {
        pixelSize: 12,
        color: Cesium.Color.RED,
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 2,
        heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
      },
      label: {
        text: "北京",
        font: "bold 14px sans-serif",
        fillColor: Cesium.Color.WHITE,
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 2,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
        pixelOffset: new Cesium.Cartesian2(0, -15),
        heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
      },
    });

    // 添加其他城市标记
    const cities = [
      { name: "上海", lon: 121.4737, lat: 31.2304 },
      { name: "广州", lon: 113.2644, lat: 23.1291 },
      { name: "深圳", lon: 114.0579, lat: 22.5431 },
      { name: "成都", lon: 104.0668, lat: 30.5728 },
      { name: "西安", lon: 108.9398, lat: 34.3416 },
    ];

    cities.forEach((city) => {
      viewer.entities.add({
        name: city.name,
        position: Cesium.Cartesian3.fromDegrees(city.lon, city.lat),
        point: {
          pixelSize: 8,
          color: Cesium.Color.ORANGE,
          outlineColor: Cesium.Color.WHITE,
          outlineWidth: 2,
          heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
        },
        label: {
          text: city.name,
          font: "12px sans-serif",
          fillColor: Cesium.Color.WHITE,
          outlineColor: Cesium.Color.BLACK,
          outlineWidth: 2,
          style: Cesium.LabelStyle.FILL_AND_OUTLINE,
          verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
          pixelOffset: new Cesium.Cartesian2(0, -10),
          heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
        },
      });
    });

    return () => {
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
        <div ref={cesiumContainerRef} className="cesium-container" />
      </main>
    </div>
  );
}

export default CesiumPage;
