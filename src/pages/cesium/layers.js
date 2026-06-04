import * as Cesium from "cesium";
import { CITIES_DATA } from "./cityData";

const SCAN_COLORS = {
  primary: "#00f5ff",
  secondary: "#8fff6a",
};

const SCAN_FOOTPRINT_RADIUS = 360000;
const SCAN_SURFACE_OFFSET = 1200;
const SCAN_SEGMENTS = 96;
const SCAN_UPDATE_INTERVAL = 0.12;

const getLoopedClockTime = (clock) => {
  const totalSeconds = Cesium.JulianDate.secondsDifference(clock.stopTime, clock.startTime);
  const elapsedSeconds = Cesium.JulianDate.secondsDifference(clock.currentTime, clock.startTime);
  const loopedSeconds = ((elapsedSeconds % totalSeconds) + totalSeconds) % totalSeconds;

  return {
    seconds: loopedSeconds,
    time: Cesium.JulianDate.addSeconds(clock.startTime, loopedSeconds, new Cesium.JulianDate()),
  };
};

const COVERAGE_STATIONS = [
  { name: "Beijing Station", lon: 116.41, lat: 39.92 },
  { name: "Shanghai Station", lon: 121.47, lat: 31.23 },
  { name: "Shenzhen Station", lon: 114.05, lat: 22.55 },
];

export const INITIAL_LAYERS = {
  satellite: true,
  flightLines: false,
  coverage: false,
  cityMarkers: true,
  provinceLabels: true,
};

// 创建卫星轨道采样点，使用地心坐标生成闭合倾斜轨道，避免跨经度变形。
const createOrbitSamples = (start, totalSeconds, orbitHeight) => {
  const position = new Cesium.SampledPositionProperty();
  const orbitPoints = [];
  const orbitRadius = Cesium.Ellipsoid.WGS84.maximumRadius + orbitHeight;
  const inclination = Cesium.Math.toRadians(48);
  const ascendingNode = Cesium.Math.toRadians(32);

  for (let i = 0; i <= totalSeconds; i += 5) {
    const angle = (i / totalSeconds) * Cesium.Math.TWO_PI;
    const orbitalX = orbitRadius * Math.cos(angle);
    const orbitalY = orbitRadius * Math.sin(angle);
    const inclinedY = orbitalY * Math.cos(inclination);
    const inclinedZ = orbitalY * Math.sin(inclination);
    const point = new Cesium.Cartesian3(
      orbitalX * Math.cos(ascendingNode) - inclinedY * Math.sin(ascendingNode),
      orbitalX * Math.sin(ascendingNode) + inclinedY * Math.cos(ascendingNode),
      inclinedZ
    );
    const time = Cesium.JulianDate.addSeconds(start, i, new Cesium.JulianDate());
    position.addSample(time, point);
    orbitPoints.push(point);
  }

  position.setInterpolationOptions({
    interpolationDegree: 3,
    interpolationAlgorithm: Cesium.HermitePolynomialApproximation,
  });

  return { position, orbitPoints };
};

// 配置 Cesium 时钟，让卫星沿轨道循环运动。
const configureSatelliteClock = (viewer, start, totalSeconds) => {
  viewer.clock.startTime = start.clone();
  viewer.clock.stopTime = Cesium.JulianDate.addSeconds(start, totalSeconds, new Cesium.JulianDate());
  viewer.clock.currentTime = start.clone();
  viewer.clock.multiplier = 8;
  viewer.clock.shouldAnimate = true;
  viewer.clock.clockRange = Cesium.ClockRange.LOOP_STOP;
};

// 创建由主体和太阳能板组成的简单卫星模型。
const createSatelliteModel = (viewer, position) => {
  const orientation = new Cesium.VelocityOrientationProperty(position);
  const body = viewer.entities.add({
    name: "LEO-01 Satellite Body",
    position,
    orientation,
    box: {
      dimensions: new Cesium.Cartesian3(150000, 86000, 86000),
      material: Cesium.Color.fromCssColorString("#dfe9ff").withAlpha(0.92),
      outline: true,
      outlineColor: Cesium.Color.fromCssColorString("#ffdc62"),
    },
  });

  const leftPanel = viewer.entities.add({
    name: "LEO-01 Left Solar Panel",
    position,
    orientation,
    box: {
      dimensions: new Cesium.Cartesian3(260000, 28000, 5600),
      material: Cesium.Color.fromCssColorString("#1f8fff").withAlpha(0.78),
      outline: true,
      outlineColor: Cesium.Color.fromCssColorString("#7de7ff"),
    },
  });

  const rightPanel = viewer.entities.add({
    name: "LEO-01 Right Solar Panel",
    position,
    orientation,
    box: {
      dimensions: new Cesium.Cartesian3(28000, 260000, 5600),
      material: Cesium.Color.fromCssColorString("#1f8fff").withAlpha(0.78),
      outline: true,
      outlineColor: Cesium.Color.fromCssColorString("#7de7ff"),
    },
  });

  const label = viewer.entities.add({
    name: "LEO-01 Satellite Label",
    position,
    label: {
      text: "LEO-01",
      font: "bold 12px sans-serif",
      fillColor: Cesium.Color.fromCssColorString("#ffdc62"),
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 2,
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset: new Cesium.Cartesian2(0, -18),
    },
  });

  return [body, leftPanel, rightPanel, label];
};

// 创建固定颜色的闭合轨道线。
const createOrbitLine = (viewer, orbitPoints) => {
  return viewer.entities.add({
    name: "Satellite Orbit",
    polyline: {
      positions: [...orbitPoints, orbitPoints[0]],
      width: 2.4,
      material: Cesium.Color.fromCssColorString("#ffdc62"),
    },
  });
};

// 根据当前卫星位置计算地表投影点，偏移量用于让两条扫描锥错开显示。
const getSatelliteGroundPoint = (position, orbitPoints, time, lonOffset = 0, latOffset = 0) => {
  const satPosition = position.getValue(time) || orbitPoints[0];
  const cartographic = Cesium.Cartographic.fromCartesian(satPosition);

  return Cesium.Cartesian3.fromRadians(
    cartographic.longitude + Cesium.Math.toRadians(lonOffset),
    Cesium.Math.clamp(
      cartographic.latitude + Cesium.Math.toRadians(latOffset),
      Cesium.Math.toRadians(-82),
      Cesium.Math.toRadians(82)
    ),
    0
  );
};

// 根据地表投影中心生成贴合地球曲面的扫描面边界点。
const createSurfaceFootprint = (groundPosition) => {
  const transform = Cesium.Transforms.eastNorthUpToFixedFrame(groundPosition);
  const points = [];

  for (let i = 0; i < SCAN_SEGMENTS; i += 1) {
    const angle = (i / SCAN_SEGMENTS) * Cesium.Math.TWO_PI;
    const tangentPoint = Cesium.Matrix4.multiplyByPoint(
      transform,
      new Cesium.Cartesian3(
        Math.cos(angle) * SCAN_FOOTPRINT_RADIUS,
        Math.sin(angle) * SCAN_FOOTPRINT_RADIUS,
        SCAN_SURFACE_OFFSET
      ),
      new Cesium.Cartesian3()
    );
    const cartographic = Cesium.Cartographic.fromCartesian(tangentPoint);
    points.push(Cesium.Cartesian3.fromRadians(
      cartographic.longitude,
      cartographic.latitude,
      SCAN_SURFACE_OFFSET
    ));
  }

  return points;
};

// 创建一个由侧面和贴地扫描面组成的整体锥体几何。
const createScanConeGeometry = (satellitePosition, groundPosition) => {
  const footprint = createSurfaceFootprint(groundPosition);
  const center = Cesium.Cartographic.fromCartesian(groundPosition);
  const surfaceCenter = Cesium.Cartesian3.fromRadians(
    center.longitude,
    center.latitude,
    SCAN_SURFACE_OFFSET
  );
  const positions = [satellitePosition, surfaceCenter, ...footprint];
  const values = new Float64Array(positions.length * 3);
  const indices = [];

  positions.forEach((point, index) => {
    values[index * 3] = point.x;
    values[index * 3 + 1] = point.y;
    values[index * 3 + 2] = point.z;
  });

  for (let i = 0; i < SCAN_SEGMENTS; i += 1) {
    const current = i + 2;
    const next = ((i + 1) % SCAN_SEGMENTS) + 2;
    indices.push(0, current, next);
    indices.push(1, next, current);
  }

  return new Cesium.Geometry({
    attributes: {
      position: new Cesium.GeometryAttribute({
        componentDatatype: Cesium.ComponentDatatype.DOUBLE,
        componentsPerAttribute: 3,
        values,
      }),
    },
    indices,
    primitiveType: Cesium.PrimitiveType.TRIANGLES,
    boundingSphere: Cesium.BoundingSphere.fromPoints(positions),
  });
};

// 用动态 Primitive 绘制一体式扫描锥，底部三角面直接贴近地球曲面。
const createScanConePrimitive = (viewer, position, orbitPoints, scan) => {
  const color = Cesium.Color.fromCssColorString(scan.color);
  let primitive = null;
  let lastUpdateSecond = Number.NEGATIVE_INFINITY;
  let visible = true;

  const removePrimitive = () => {
    if (primitive && !viewer.isDestroyed()) {
      viewer.scene.primitives.remove(primitive);
      primitive = null;
    }
  };

  const render = (clock) => {
    if (!visible || viewer.isDestroyed()) {
      removePrimitive();
      return;
    }

    const { seconds: currentSecond, time: loopedTime } = getLoopedClockTime(clock);
    if (currentSecond < lastUpdateSecond) {
      lastUpdateSecond = Number.NEGATIVE_INFINITY;
    }
    if (currentSecond - lastUpdateSecond < SCAN_UPDATE_INTERVAL) return;
    lastUpdateSecond = currentSecond;

    const satellitePosition = position.getValue(loopedTime) || position.getValue(clock.startTime) || orbitPoints[0];
    const groundPosition = getSatelliteGroundPoint(
      position,
      orbitPoints,
      loopedTime,
      scan.lonOffset,
      scan.latOffset
    );

    removePrimitive();
    primitive = viewer.scene.primitives.add(new Cesium.Primitive({
      geometryInstances: new Cesium.GeometryInstance({
        geometry: createScanConeGeometry(satellitePosition, groundPosition),
        attributes: {
          color: Cesium.ColorGeometryInstanceAttribute.fromColor(color.withAlpha(0.22)),
        },
      }),
      appearance: new Cesium.PerInstanceColorAppearance({
        flat: true,
        translucent: true,
        closed: true,
      }),
      asynchronous: false,
    }));
  };

  viewer.clock.onTick.addEventListener(render);
  render(viewer.clock);

  return {
    name: scan.name,
    get show() {
      return visible;
    },
    set show(nextVisible) {
      visible = nextVisible;
      if (primitive) {
        primitive.show = nextVisible;
      }
      if (!nextVisible) {
        removePrimitive();
      } else {
        lastUpdateSecond = Number.NEGATIVE_INFINITY;
        render(viewer.clock);
      }
    },
  };
};

// 创建两条随卫星移动的一体式扫描锥。
const createMovingScanCones = (viewer, position, orbitPoints) => {
  return [
    { name: "Primary Moving Scan Cone", color: SCAN_COLORS.primary, lonOffset: -6, latOffset: 2 },
    { name: "Secondary Moving Scan Cone", color: SCAN_COLORS.secondary, lonOffset: 6, latOffset: -2 },
  ].map((scan) => createScanConePrimitive(viewer, position, orbitPoints, scan));
};

// 创建卫星模型、闭合轨道线，以及两条跟随卫星移动的地表扫描锥。
export const createSatelliteTrack = (viewer) => {
  const start = Cesium.JulianDate.now();
  const totalSeconds = 720;
  const orbitHeight = 900000;
  const { position, orbitPoints } = createOrbitSamples(start, totalSeconds, orbitHeight);

  configureSatelliteClock(viewer, start, totalSeconds);

  return [
    ...createSatelliteModel(viewer, position),
    createOrbitLine(viewer, orbitPoints),
    ...createMovingScanCones(viewer, position, orbitPoints),
  ];
};

// 创建城市之间的高空飞线，用于展示城市间数据或航路连接。
export const createFlightLines = (viewer) => {
  const routes = [
    [CITIES_DATA[0], CITIES_DATA[1]],
    [CITIES_DATA[1], CITIES_DATA[2]],
    [CITIES_DATA[2], CITIES_DATA[4]],
    [CITIES_DATA[3], CITIES_DATA[5]],
  ].filter(([from, to]) => from && to);

  return routes.map(([from, to]) =>
    viewer.entities.add({
      name: `${from.name}-${to.name}`,
      polyline: {
        positions: Cesium.Cartesian3.fromDegreesArrayHeights([
          from.lon,
          from.lat,
          20000,
          (from.lon + to.lon) / 2,
          (from.lat + to.lat) / 2,
          550000,
          to.lon,
          to.lat,
          20000,
        ]),
        width: 2,
        material: Cesium.Color.fromCssColorString("#bf00ff").withAlpha(0.65),
      },
    })
  );
};

// 创建地面站覆盖范围圆，用于展示站点可覆盖的地表区域。
export const createCoverageAreas = (viewer) => {
  return COVERAGE_STATIONS.map((station) =>
    viewer.entities.add({
      name: `${station.name} Coverage`,
      position: Cesium.Cartesian3.fromDegrees(station.lon, station.lat),
      ellipse: {
        semiMajorAxis: 650000,
        semiMinorAxis: 650000,
        height: 0,
        material: Cesium.Color.fromCssColorString("#00f5ff").withAlpha(0.12),
        outline: true,
        outlineColor: Cesium.Color.fromCssColorString("#00f5ff").withAlpha(0.65),
      },
    })
  );
};
