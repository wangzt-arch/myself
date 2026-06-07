import * as Cesium from "cesium";

export const DRONE_ROUTE_OPTIONS = [
  { key: "city", label: "城市巡检" },
  { key: "river", label: "河道巡检" },
  { key: "park", label: "园区巡航" },
];

const DRONE_ROUTES = {
  city: {
    label: "城市巡检",
    height: 260,
    speed: "42km/h",
    duration: 80,
    points: [
      { lon: 116.378, lat: 39.902 },
      { lon: 116.397, lat: 39.914 },
      { lon: 116.421, lat: 39.906 },
      { lon: 116.415, lat: 39.887 },
      { lon: 116.389, lat: 39.883 },
    ],
  },
  river: {
    label: "河道巡检",
    height: 180,
    speed: "34km/h",
    duration: 92,
    points: [
      { lon: 121.438, lat: 31.202 },
      { lon: 121.458, lat: 31.218 },
      { lon: 121.484, lat: 31.225 },
      { lon: 121.507, lat: 31.239 },
      { lon: 121.528, lat: 31.248 },
    ],
  },
  park: {
    label: "园区巡航",
    height: 120,
    speed: "28km/h",
    duration: 70,
    points: [
      { lon: 113.934, lat: 22.533 },
      { lon: 113.948, lat: 22.542 },
      { lon: 113.965, lat: 22.536 },
      { lon: 113.961, lat: 22.519 },
      { lon: 113.941, lat: 22.516 },
    ],
  },
};

const getRoute = (routeKey) => DRONE_ROUTES[routeKey] || DRONE_ROUTES.city;

const getPointWithHeight = (point, route) => ({
  lon: point.lon,
  lat: point.lat,
  height: point.height ?? route.height,
});

const toCartesian = (point, route) => {
  const finalPoint = getPointWithHeight(point, route);
  return Cesium.Cartesian3.fromDegrees(finalPoint.lon, finalPoint.lat, finalPoint.height);
};

const getSegmentLengths = (route) => {
  const positions = route.points.map((point) => toCartesian(point, route));
  const closedPositions = [...positions, positions[0]];
  const lengths = [];
  let total = 0;

  for (let index = 0; index < closedPositions.length - 1; index += 1) {
    const length = Cesium.Cartesian3.distance(closedPositions[index], closedPositions[index + 1]);
    lengths.push(length);
    total += length;
  }

  return { positions, lengths, total };
};

const getElapsedSeconds = (controller) => {
  if (controller.status !== "flying" || controller.startedAt == null) {
    return controller.elapsedSeconds;
  }

  return controller.elapsedSeconds + (Date.now() - controller.startedAt) / 1000;
};

const getRoutePositionByProgress = (route, progress) => {
  const { positions, lengths, total } = getSegmentLengths(route);
  let distance = progress * total;

  for (let index = 0; index < lengths.length; index += 1) {
    if (distance <= lengths[index]) {
      const amount = lengths[index] === 0 ? 0 : distance / lengths[index];
      return Cesium.Cartesian3.lerp(
        positions[index],
        positions[(index + 1) % positions.length],
        amount,
        new Cesium.Cartesian3()
      );
    }
    distance -= lengths[index];
  }

  return Cesium.Cartesian3.clone(positions[0]);
};

const getCurrentDronePosition = (controller, offsetSeconds = 0) => {
  const elapsed = Math.max(0, getElapsedSeconds(controller) - offsetSeconds);
  const progress = (elapsed % controller.route.duration) / controller.route.duration;
  return getRoutePositionByProgress(controller.route, progress);
};

const getGroundPosition = (position) => {
  const cartographic = Cesium.Cartographic.fromCartesian(position);
  return Cesium.Cartesian3.fromRadians(cartographic.longitude, cartographic.latitude, 8);
};

const createTrailPositions = (controller) => {
  const samples = [];
  for (let index = 12; index >= 0; index -= 1) {
    samples.push(getCurrentDronePosition(controller, index * 1.2));
  }
  return samples;
};

const getVectorColumn = (matrix, index) => {
  const column = Cesium.Matrix4.getColumn(matrix, index, new Cesium.Cartesian4());
  return new Cesium.Cartesian3(column.x, column.y, column.z);
};

const getDroneHeading = (controller) => {
  const current = getCurrentDronePosition(controller);
  const next = getCurrentDronePosition(controller, -0.4);
  const transform = Cesium.Transforms.eastNorthUpToFixedFrame(current);
  const inverse = Cesium.Matrix4.inverseTransformation(transform, new Cesium.Matrix4());
  const localNext = Cesium.Matrix4.multiplyByPoint(inverse, next, new Cesium.Cartesian3());

  return Math.atan2(localNext.x, localNext.y);
};

const getDroneOrientation = (controller) => {
  const position = getCurrentDronePosition(controller);
  const heading = getDroneHeading(controller);
  return Cesium.Transforms.headingPitchRollQuaternion(
    position,
    new Cesium.HeadingPitchRoll(heading, 0, 0)
  );
};

const getDroneOffsetPosition = (controller, forwardOffset, rightOffset, upOffset = 0) => {
  const position = getCurrentDronePosition(controller);
  const heading = getDroneHeading(controller);
  const transform = Cesium.Transforms.eastNorthUpToFixedFrame(position);
  const east = Cesium.Cartesian3.normalize(getVectorColumn(transform, 0), new Cesium.Cartesian3());
  const north = Cesium.Cartesian3.normalize(getVectorColumn(transform, 1), new Cesium.Cartesian3());
  const up = Cesium.Cartesian3.normalize(getVectorColumn(transform, 2), new Cesium.Cartesian3());
  const forward = Cesium.Cartesian3.add(
    Cesium.Cartesian3.multiplyByScalar(north, Math.cos(heading), new Cesium.Cartesian3()),
    Cesium.Cartesian3.multiplyByScalar(east, Math.sin(heading), new Cesium.Cartesian3()),
    new Cesium.Cartesian3()
  );
  const right = Cesium.Cartesian3.add(
    Cesium.Cartesian3.multiplyByScalar(east, Math.cos(heading), new Cesium.Cartesian3()),
    Cesium.Cartesian3.multiplyByScalar(north, -Math.sin(heading), new Cesium.Cartesian3()),
    new Cesium.Cartesian3()
  );
  const forwardVector = Cesium.Cartesian3.multiplyByScalar(forward, forwardOffset, new Cesium.Cartesian3());
  const rightVector = Cesium.Cartesian3.multiplyByScalar(right, rightOffset, new Cesium.Cartesian3());
  const upVector = Cesium.Cartesian3.multiplyByScalar(up, upOffset, new Cesium.Cartesian3());

  return Cesium.Cartesian3.add(
    position,
    Cesium.Cartesian3.add(forwardVector, Cesium.Cartesian3.add(rightVector, upVector, new Cesium.Cartesian3()), new Cesium.Cartesian3()),
    new Cesium.Cartesian3()
  );
};

const createDroneModelEntities = (viewer, controller, dronePosition, droneOrientation, routeColor) => {
  const bodyMaterial = Cesium.Color.fromCssColorString("#dffdf6").withAlpha(0.96);
  const armMaterial = Cesium.Color.fromCssColorString("#5f7686").withAlpha(0.95);
  const rotorMaterial = Cesium.Color.fromCssColorString("#152932").withAlpha(0.78);
  const glowMaterial = routeColor.withAlpha(0.7);

  const body = viewer.entities.add({
    name: "UAV-01 Body",
    position: dronePosition,
    orientation: droneOrientation,
    box: {
      dimensions: new Cesium.Cartesian3(78, 28, 18),
      material: bodyMaterial,
      outline: true,
      outlineColor: routeColor,
    },
  });

  const sideArm = viewer.entities.add({
    name: "UAV-01 Side Arm",
    position: dronePosition,
    orientation: droneOrientation,
    box: {
      dimensions: new Cesium.Cartesian3(150, 8, 6),
      material: armMaterial,
    },
  });

  const frontArm = viewer.entities.add({
    name: "UAV-01 Front Arm",
    position: dronePosition,
    orientation: droneOrientation,
    box: {
      dimensions: new Cesium.Cartesian3(8, 126, 6),
      material: armMaterial,
    },
  });

  const rotorOffsets = [
    [50, 62],
    [50, -62],
    [-50, 62],
    [-50, -62],
  ];
  const rotors = rotorOffsets.map(([forward, right], index) =>
    viewer.entities.add({
      name: `UAV-01 Rotor-${index + 1}`,
      position: new Cesium.CallbackProperty(() => getDroneOffsetPosition(controller, forward, right, 5), false),
      orientation: droneOrientation,
      cylinder: {
        length: 5,
        topRadius: 24,
        bottomRadius: 24,
        material: rotorMaterial,
        outline: true,
        outlineColor: glowMaterial,
      },
    })
  );

  const label = viewer.entities.add({
    name: "UAV-01 Label",
    position: new Cesium.CallbackProperty(() => getDroneOffsetPosition(controller, 0, 0, 34), false),
    label: {
      text: "UAV-01",
      font: "bold 12px sans-serif",
      fillColor: routeColor,
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 2,
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset: new Cesium.Cartesian2(0, -24),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
  });

  return [body, sideArm, frontArm, ...rotors, label];
};

const addDroneEntities = (viewer, controller) => {
  const routePositions = controller.route.points.map((point) => toCartesian(point, controller.route));
  const closedRoute = [...routePositions, routePositions[0]];
  const dronePosition = new Cesium.CallbackProperty(() => getCurrentDronePosition(controller), false);
  const droneOrientation = new Cesium.CallbackProperty(() => getDroneOrientation(controller), false);
  const routeColor = Cesium.Color.fromCssColorString("#67f6c3");

  const routeLine = viewer.entities.add({
    name: "UAV Low Altitude Route",
    polyline: {
      positions: closedRoute,
      width: 3,
      material: routeColor.withAlpha(0.78),
    },
  });

  const trailLine = viewer.entities.add({
    name: "UAV Flight Trail",
    polyline: {
      positions: new Cesium.CallbackProperty(() => createTrailPositions(controller), false),
      width: 5,
      material: Cesium.Color.fromCssColorString("#9fffe2").withAlpha(0.72),
    },
  });

  const droneModel = createDroneModelEntities(viewer, controller, dronePosition, droneOrientation, routeColor);

  const scanArea = viewer.entities.add({
    name: "UAV Ground Scan",
    position: new Cesium.CallbackProperty(() => getGroundPosition(getCurrentDronePosition(controller)), false),
    ellipse: {
      semiMajorAxis: 850,
      semiMinorAxis: 850,
      height: 6,
      material: routeColor.withAlpha(0.2),
      outline: true,
      outlineColor: routeColor.withAlpha(0.78),
    },
  });

  const waypoints = controller.route.points.map((point, index) =>
    viewer.entities.add({
      name: `UAV WP-${String(index + 1).padStart(2, "0")}`,
      position: toCartesian(point, controller.route),
      point: {
        pixelSize: 9,
        color: routeColor.withAlpha(0.95),
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 1,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
      label: {
        text: `WP-${index + 1}`,
        font: "11px sans-serif",
        fillColor: Cesium.Color.WHITE,
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 2,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new Cesium.Cartesian2(0, -16),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    })
  );

  controller.routeEntities = [routeLine, trailLine, ...waypoints];
  controller.droneEntities = droneModel;
  controller.scanEntities = [scanArea];
  controller.entities = [...controller.routeEntities, ...controller.droneEntities, ...controller.scanEntities];
  controller.routeEntities.forEach((entity) => {
    entity.show = controller.visible;
  });
  [...controller.droneEntities, ...controller.scanEntities].forEach((entity) => {
    entity.show = true;
  });
};

const removeDroneEntities = (controller) => {
  controller.entities.forEach((entity) => controller.viewer.entities.remove(entity));
  controller.entities = [];
  controller.routeEntities = [];
  controller.droneEntities = [];
  controller.scanEntities = [];
};

const refreshDroneRoute = (controller, routeKey) => {
  removeDroneEntities(controller);
  controller.routeKey = routeKey;
  controller.route = getRoute(routeKey);
  controller.elapsedSeconds = 0;
  controller.startedAt = null;
  controller.status = "ready";
  addDroneEntities(controller.viewer, controller);
};

export const createDronePatrol = (viewer, routeKey = "city") => {
  const controller = {
    viewer,
    routeKey,
    route: getRoute(routeKey),
    status: "ready",
    visible: true,
    following: false,
    startedAt: null,
    elapsedSeconds: 0,
    entities: [],
    routeEntities: [],
    droneEntities: [],
    scanEntities: [],
  };

  controller.start = () => {
    if (controller.status !== "flying") {
      controller.startedAt = Date.now();
      controller.status = "flying";
    }
  };

  controller.pause = () => {
    if (controller.status === "flying") {
      controller.elapsedSeconds = getElapsedSeconds(controller);
      controller.startedAt = null;
      controller.status = "paused";
    }
  };

  controller.reset = () => {
    controller.elapsedSeconds = 0;
    controller.startedAt = null;
    controller.status = "ready";
  };

  controller.setVisible = (visible) => {
    controller.visible = visible;
    controller.routeEntities.forEach((entity) => {
      entity.show = visible;
    });
  };

  controller.setRoute = (nextRouteKey) => {
    refreshDroneRoute(controller, nextRouteKey);
  };

  controller.remove = () => {
    removeDroneEntities(controller);
  };

  controller.getPosition = () => getCurrentDronePosition(controller);
  controller.getProgress = () => Math.round(((getElapsedSeconds(controller) % controller.route.duration) / controller.route.duration) * 100);
  controller.getSnapshot = () => ({
    status: controller.status,
    routeKey: controller.routeKey,
    routeLabel: controller.route.label,
    height: controller.route.height,
    speed: controller.route.speed,
    progress: controller.getProgress(),
    visible: controller.visible,
    following: controller.following,
  });

  addDroneEntities(viewer, controller);
  return controller;
};

export const updateDroneFollowCamera = (viewer, controller) => {
  if (!controller?.following) return;

  viewer.camera.lookAt(
    controller.getPosition(),
    new Cesium.HeadingPitchRange(
      Cesium.Math.toRadians(0),
      Cesium.Math.toRadians(-90),
      5200
    )
  );
};
