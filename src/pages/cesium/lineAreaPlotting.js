import * as Cesium from "cesium";

export const PLOTTING_TYPES = {
  STRAIGHT_LINE: "straightLine",
  POLYLINE: "polyline",
  CURVE: "curve",
  FREE_LINE: "freeLine",
  ARC: "arc",
  ATTACK_ARROW: "attackArrow",
  PINCER_ARROW: "pincerArrow",
  POLYGON: "polygon",
  RECTANGLE: "rectangle",
  FREE_AREA: "freeArea",
  CLOSED_CURVE: "closedCurve",
};

export const PLOTTING_TOOL_GROUPS = [
  {
    label: "线标绘",
    tools: [
      [PLOTTING_TYPES.STRAIGHT_LINE, "直线"],
      [PLOTTING_TYPES.POLYLINE, "折线"],
      [PLOTTING_TYPES.CURVE, "曲线"],
      [PLOTTING_TYPES.FREE_LINE, "自由线"],
      [PLOTTING_TYPES.ARC, "弧线"],
      [PLOTTING_TYPES.ATTACK_ARROW, "进攻箭头"],
      [PLOTTING_TYPES.PINCER_ARROW, "钳击箭头"],
    ],
  },
  {
    label: "面标绘",
    tools: [
      [PLOTTING_TYPES.POLYGON, "多边形"],
      [PLOTTING_TYPES.RECTANGLE, "矩形"],
      [PLOTTING_TYPES.FREE_AREA, "自由面"],
      [PLOTTING_TYPES.CLOSED_CURVE, "闭合曲线"],
    ],
  },
];

const LINE_COLOR = Cesium.Color.fromCssColorString("#ffd166");
const AREA_COLOR = Cesium.Color.fromCssColorString("#ff7b72");
const PINCER_COLOR = Cesium.Color.fromCssColorString("#6dbb4a");

const TYPE_META = {
  [PLOTTING_TYPES.STRAIGHT_LINE]: { minimumPoints: 2, maximumPoints: 2, category: "line", name: "Straight Line" },
  [PLOTTING_TYPES.POLYLINE]: { minimumPoints: 2, category: "line", name: "Polyline" },
  [PLOTTING_TYPES.CURVE]: { minimumPoints: 3, category: "line", name: "Curve" },
  [PLOTTING_TYPES.FREE_LINE]: { minimumPoints: 2, category: "line", freehand: true, name: "Free Line" },
  [PLOTTING_TYPES.ARC]: { minimumPoints: 3, maximumPoints: 3, category: "line", name: "Arc" },
  [PLOTTING_TYPES.ATTACK_ARROW]: { minimumPoints: 3, maximumPoints: 3, category: "arrow", name: "Attack Arrow" },
  [PLOTTING_TYPES.PINCER_ARROW]: { minimumPoints: 4, maximumPoints: 4, category: "arrow", name: "Pincer Arrow" },
  [PLOTTING_TYPES.POLYGON]: { minimumPoints: 3, category: "area", name: "Polygon" },
  [PLOTTING_TYPES.RECTANGLE]: { minimumPoints: 2, maximumPoints: 2, category: "rectangle", name: "Rectangle" },
  [PLOTTING_TYPES.FREE_AREA]: { minimumPoints: 3, category: "area", freehand: true, name: "Free Area" },
  [PLOTTING_TYPES.CLOSED_CURVE]: { minimumPoints: 3, category: "area", name: "Closed Curve" },
};

export function getPlottingMinimumPoints(type) {
  return TYPE_META[type]?.minimumPoints || 2;
}

function getGroundPosition(viewer, screenPosition) {
  const ray = viewer.camera.getPickRay(screenPosition);
  const position = ray ? viewer.scene.globe.pick(ray, viewer.scene) : null;
  return position || viewer.camera.pickEllipsoid(screenPosition, viewer.scene.globe.ellipsoid);
}

function toSurface(position) {
  return Cesium.Ellipsoid.WGS84.scaleToGeodeticSurface(position, new Cesium.Cartesian3()) || position;
}

function midpoint(first, second) {
  return toSurface(Cesium.Cartesian3.midpoint(first, second, new Cesium.Cartesian3()));
}

function quadraticPoint(first, control, last, t) {
  const a = (1 - t) * (1 - t);
  const b = 2 * (1 - t) * t;
  const c = t * t;
  return toSurface(new Cesium.Cartesian3(
    first.x * a + control.x * b + last.x * c,
    first.y * a + control.y * b + last.y * c,
    first.z * a + control.z * b + last.z * c,
  ));
}

function smoothPositions(positions, closed = false) {
  if (positions.length < 3) return positions.slice();
  const result = [];
  const count = positions.length;

  if (!closed) result.push(positions[0]);
  const startIndex = closed ? 0 : 1;
  const endIndex = closed ? count : count - 1;
  for (let index = startIndex; index < endIndex; index += 1) {
    const current = positions[index];
    const previous = positions[(index - 1 + count) % count];
    const next = positions[(index + 1) % count];
    const first = midpoint(previous, current);
    const last = midpoint(current, next);
    for (let step = 0; step < 8; step += 1) {
      result.push(quadraticPoint(first, current, last, step / 8));
    }
  }
  if (!closed) result.push(positions[count - 1]);
  return result;
}

function getLinePositions(type, positions) {
  if (type === PLOTTING_TYPES.STRAIGHT_LINE) return positions.slice(0, 2);
  if (type === PLOTTING_TYPES.CURVE || type === PLOTTING_TYPES.ARC) return smoothPositions(positions);
  return positions.slice();
}

function getAreaPositions(type, positions) {
  return type === PLOTTING_TYPES.CLOSED_CURVE ? smoothPositions(positions, true) : positions.slice();
}

function createArrowPolygon(start, end) {
  const frame = Cesium.Transforms.eastNorthUpToFixedFrame(start);
  const inverseFrame = Cesium.Matrix4.inverseTransformation(frame, new Cesium.Matrix4());
  const localStart = Cesium.Matrix4.multiplyByPoint(inverseFrame, start, new Cesium.Cartesian3());
  const localEnd = Cesium.Matrix4.multiplyByPoint(inverseFrame, end, new Cesium.Cartesian3());
  const dx = localEnd.x - localStart.x;
  const dy = localEnd.y - localStart.y;
  const length = Math.sqrt(dx * dx + dy * dy);
  if (length < 1) return [];

  const ux = dx / length;
  const uy = dy / length;
  const nx = -uy;
  const ny = ux;
  const shaftHalfWidth = Math.max(30, length * 0.035);
  const headLength = Math.min(length * 0.42, shaftHalfWidth * 4.5);
  const headHalfWidth = shaftHalfWidth * 2.5;
  const neck = new Cesium.Cartesian3(
    localEnd.x - ux * headLength,
    localEnd.y - uy * headLength,
    0,
  );
  const localPoints = [
    new Cesium.Cartesian3(localStart.x + nx * shaftHalfWidth, localStart.y + ny * shaftHalfWidth, 0),
    new Cesium.Cartesian3(neck.x + nx * shaftHalfWidth, neck.y + ny * shaftHalfWidth, 0),
    new Cesium.Cartesian3(neck.x + nx * headHalfWidth, neck.y + ny * headHalfWidth, 0),
    localEnd,
    new Cesium.Cartesian3(neck.x - nx * headHalfWidth, neck.y - ny * headHalfWidth, 0),
    new Cesium.Cartesian3(neck.x - nx * shaftHalfWidth, neck.y - ny * shaftHalfWidth, 0),
    new Cesium.Cartesian3(localStart.x - nx * shaftHalfWidth, localStart.y - ny * shaftHalfWidth, 0),
  ];

  return localPoints.map((point) => {
    const worldPoint = Cesium.Matrix4.multiplyByPoint(frame, point, new Cesium.Cartesian3());
    return toSurface(worldPoint);
  });
}

function createCurvedAttackArrowPolygon(start, control, end) {
  const frame = Cesium.Transforms.eastNorthUpToFixedFrame(start);
  const inverseFrame = Cesium.Matrix4.inverseTransformation(frame, new Cesium.Matrix4());
  const localStart = Cesium.Matrix4.multiplyByPoint(inverseFrame, start, new Cesium.Cartesian3());
  const localControl = Cesium.Matrix4.multiplyByPoint(inverseFrame, control, new Cesium.Cartesian3());
  const localEnd = Cesium.Matrix4.multiplyByPoint(inverseFrame, end, new Cesium.Cartesian3());
  const samplePoint = (t) => {
    const inverseT = 1 - t;
    return new Cesium.Cartesian3(
      inverseT * inverseT * localStart.x + 2 * inverseT * t * localControl.x + t * t * localEnd.x,
      inverseT * inverseT * localStart.y + 2 * inverseT * t * localControl.y + t * t * localEnd.y,
      0,
    );
  };
  const tangentAt = (t) => ({
    x: 2 * (1 - t) * (localControl.x - localStart.x) + 2 * t * (localEnd.x - localControl.x),
    y: 2 * (1 - t) * (localControl.y - localStart.y) + 2 * t * (localEnd.y - localControl.y),
  });
  const routeLength = Cesium.Cartesian3.distance(localStart, localControl) + Cesium.Cartesian3.distance(localControl, localEnd);
  if (routeLength < 1) return [];

  // Keep the shaft restrained so the tactical arrow reads as a directional mark,
  // rather than a broad ribbon when the middle control point introduces a bend.
  const shaftHalfWidth = Math.max(16, routeLength * 0.026);
  const leftSide = [];
  const rightSide = [];
  const headStart = 0.74;
  for (let index = 0; index <= 18; index += 1) {
    const t = index / 18 * headStart;
    const point = samplePoint(t);
    const tangent = tangentAt(t);
    const tangentLength = Math.hypot(tangent.x, tangent.y) || 1;
    const normalX = -tangent.y / tangentLength;
    const normalY = tangent.x / tangentLength;
    const width = shaftHalfWidth * (1 - t * 0.1);
    leftSide.push(new Cesium.Cartesian3(point.x + normalX * width, point.y + normalY * width, 0));
    rightSide.push(new Cesium.Cartesian3(point.x - normalX * width, point.y - normalY * width, 0));
  }

  const headBase = samplePoint(headStart);
  const headTangent = tangentAt(headStart);
  const headTangentLength = Math.hypot(headTangent.x, headTangent.y) || 1;
  const headNormalX = -headTangent.y / headTangentLength;
  const headNormalY = headTangent.x / headTangentLength;
  const headHalfWidth = shaftHalfWidth * 1.4;
  const localPolygon = [
    ...leftSide,
    new Cesium.Cartesian3(headBase.x + headNormalX * headHalfWidth, headBase.y + headNormalY * headHalfWidth, 0),
    localEnd,
    new Cesium.Cartesian3(headBase.x - headNormalX * headHalfWidth, headBase.y - headNormalY * headHalfWidth, 0),
    ...rightSide.reverse(),
  ];

  return localPolygon.map((point) => {
    const worldPoint = Cesium.Matrix4.multiplyByPoint(frame, point, new Cesium.Cartesian3());
    return toSurface(worldPoint);
  });
}

function sampleQuadratic(first, control, last, steps) {
  const points = [];
  for (let index = 0; index <= steps; index += 1) {
    const t = index / steps;
    const oneMinusT = 1 - t;
    points.push(new Cesium.Cartesian3(
      oneMinusT * oneMinusT * first.x + 2 * oneMinusT * t * control.x + t * t * last.x,
      oneMinusT * oneMinusT * first.y + 2 * oneMinusT * t * control.y + t * t * last.y,
      0,
    ));
  }
  return points;
}

function unitVector(from, to, fallback) {
  const x = to.x - from.x;
  const y = to.y - from.y;
  const length = Math.sqrt(x * x + y * y);
  return length > 1 ? { x: x / length, y: y / length } : fallback;
}

function createIntegratedPincerPolygon(leftBase, rightBase, leftTip, rightTip) {
  const origin = midpoint(leftBase, rightBase);
  const frame = Cesium.Transforms.eastNorthUpToFixedFrame(origin);
  const inverseFrame = Cesium.Matrix4.inverseTransformation(frame, new Cesium.Matrix4());
  const toLocal = (position) => Cesium.Matrix4.multiplyByPoint(inverseFrame, position, new Cesium.Cartesian3());
  const localLeftBase = toLocal(leftBase);
  const localRightBase = toLocal(rightBase);
  const localLeftTip = toLocal(leftTip);
  const localRightTip = toLocal(rightTip);
  const baseCenter = new Cesium.Cartesian3(
    (localLeftBase.x + localRightBase.x) / 2,
    (localLeftBase.y + localRightBase.y) / 2,
    0,
  );
  const tipCenter = new Cesium.Cartesian3(
    (localLeftTip.x + localRightTip.x) / 2,
    (localLeftTip.y + localRightTip.y) / 2,
    0,
  );
  const forward = unitVector(baseCenter, tipCenter, { x: 0, y: -1 });
  const leftOut = unitVector(baseCenter, localLeftBase, { x: -forward.y, y: forward.x });
  const rightOut = unitVector(baseCenter, localRightBase, { x: forward.y, y: -forward.x });
  const span = Math.max(80, Cesium.Cartesian3.distance(localLeftBase, localRightBase));
  const outerLift = { x: -forward.x * span * 1.2, y: -forward.y * span * 1.2 };
  const innerLift = { x: forward.x * span * 0.55, y: forward.y * span * 0.55 };
  const headLength = span * 0.5;
  const headHalfWidth = span * 0.28;
  const leftDirection = unitVector(localLeftBase, localLeftTip, forward);
  const rightDirection = unitVector(localRightBase, localRightTip, forward);
  const leftHeadBase = new Cesium.Cartesian3(
    localLeftTip.x - leftDirection.x * headLength,
    localLeftTip.y - leftDirection.y * headLength,
    0,
  );
  const rightHeadBase = new Cesium.Cartesian3(
    localRightTip.x - rightDirection.x * headLength,
    localRightTip.y - rightDirection.y * headLength,
    0,
  );
  const leftOuterHead = new Cesium.Cartesian3(
    leftHeadBase.x + leftOut.x * headHalfWidth,
    leftHeadBase.y + leftOut.y * headHalfWidth,
    0,
  );
  const leftInnerHead = new Cesium.Cartesian3(
    leftHeadBase.x - leftOut.x * headHalfWidth,
    leftHeadBase.y - leftOut.y * headHalfWidth,
    0,
  );
  const rightOuterHead = new Cesium.Cartesian3(
    rightHeadBase.x + rightOut.x * headHalfWidth,
    rightHeadBase.y + rightOut.y * headHalfWidth,
    0,
  );
  const rightInnerHead = new Cesium.Cartesian3(
    rightHeadBase.x - rightOut.x * headHalfWidth,
    rightHeadBase.y - rightOut.y * headHalfWidth,
    0,
  );
  const outerBridgeControl = new Cesium.Cartesian3(baseCenter.x + outerLift.x, baseCenter.y + outerLift.y, 0);
  const innerBridgeControl = new Cesium.Cartesian3(baseCenter.x + innerLift.x, baseCenter.y + innerLift.y, 0);
  const rightArmControl = new Cesium.Cartesian3(
    localRightBase.x + outerLift.x * 0.65 + rightOut.x * span * 0.45,
    localRightBase.y + outerLift.y * 0.65 + rightOut.y * span * 0.45,
    0,
  );
  const leftArmControl = new Cesium.Cartesian3(
    localLeftBase.x + outerLift.x * 0.65 + leftOut.x * span * 0.45,
    localLeftBase.y + outerLift.y * 0.65 + leftOut.y * span * 0.45,
    0,
  );
  const localPolygon = [
    ...sampleQuadratic(localLeftBase, outerBridgeControl, localRightBase, 8),
    ...sampleQuadratic(localRightBase, rightArmControl, rightOuterHead, 8).slice(1),
    localRightTip,
    rightInnerHead,
    ...sampleQuadratic(rightInnerHead, innerBridgeControl, leftInnerHead, 12).slice(1),
    localLeftTip,
    leftOuterHead,
    ...sampleQuadratic(leftOuterHead, leftArmControl, localLeftBase, 8).slice(1),
  ];

  return localPolygon.map((point) => {
    const worldPoint = Cesium.Matrix4.multiplyByPoint(frame, point, new Cesium.Cartesian3());
    return toSurface(worldPoint);
  });
}

function reflectPincerTip(leftBase, rightBase, tip) {
  const origin = midpoint(leftBase, rightBase);
  const frame = Cesium.Transforms.eastNorthUpToFixedFrame(origin);
  const inverseFrame = Cesium.Matrix4.inverseTransformation(frame, new Cesium.Matrix4());
  const toPoint = (position) => {
    const local = Cesium.Matrix4.multiplyByPoint(inverseFrame, position, new Cesium.Cartesian3());
    return { x: local.x, y: local.y };
  };
  const first = toPoint(leftBase);
  const second = toPoint(rightBase);
  const third = toPoint(tip);
  const center = pointMid(first, second);
  const length = pointDistance(center, third);
  const angle = pointAngle(first, center, third);
  let offsetA;
  let offsetB;
  let bridge;
  let fourth;

  if (angle < Math.PI / 2) {
    offsetA = length * Math.sin(angle);
    offsetB = length * Math.cos(angle);
    bridge = pointOffset(first, center, Math.PI / 2, offsetA, false);
    fourth = pointOffset(center, bridge, Math.PI / 2, offsetB, true);
  } else if (angle < Math.PI) {
    offsetA = length * Math.sin(Math.PI - angle);
    offsetB = length * Math.cos(Math.PI - angle);
    bridge = pointOffset(first, center, Math.PI / 2, offsetA, false);
    fourth = pointOffset(center, bridge, Math.PI / 2, offsetB, false);
  } else if (angle < Math.PI * 1.5) {
    offsetA = length * Math.sin(angle - Math.PI);
    offsetB = length * Math.cos(angle - Math.PI);
    bridge = pointOffset(first, center, Math.PI / 2, offsetA, true);
    fourth = pointOffset(center, bridge, Math.PI / 2, offsetB, true);
  } else {
    offsetA = length * Math.sin(Math.PI * 2 - angle);
    offsetB = length * Math.cos(Math.PI * 2 - angle);
    bridge = pointOffset(first, center, Math.PI / 2, offsetA, true);
    fourth = pointOffset(center, bridge, Math.PI / 2, offsetB, false);
  }

  const worldPoint = Cesium.Matrix4.multiplyByPoint(frame, new Cesium.Cartesian3(fourth.x, fourth.y, 0), new Cesium.Cartesian3());
  return toSurface(worldPoint);
}

function pointDistance(first, second) {
  return Math.hypot(first.x - second.x, first.y - second.y) || 0.001;
}

function pointMid(first, second) {
  return { x: (first.x + second.x) / 2, y: (first.y + second.y) / 2 };
}

function pointOffset(start, end, angle, distance, clockwise) {
  // Standard military-plot geometry measures the direction from the end point back to the start point.
  const azimuth = Math.atan2(start.y - end.y, start.x - end.x);
  const direction = clockwise ? azimuth + angle : azimuth - angle;
  return {
    x: end.x + distance * Math.cos(direction),
    y: end.y + distance * Math.sin(direction),
  };
}

function pointAngle(first, middle, last) {
  let angle = Math.atan2(first.y - middle.y, first.x - middle.x) - Math.atan2(last.y - middle.y, last.x - middle.x);
  if (angle < 0) angle += Math.PI * 2;
  return angle;
}

function isClockwise(first, second, last) {
  return (second.x - first.x) * (last.y - first.y) - (second.y - first.y) * (last.x - first.x) > 0;
}

function bezierPoints(points, steps = 28) {
  if (points.length < 3) return points.slice();
  const n = points.length - 1;
  const factorial = (value) => {
    let result = 1;
    for (let index = 2; index <= value; index += 1) result *= index;
    return result;
  };
  const result = [];
  for (let step = 0; step <= steps; step += 1) {
    const t = step / steps;
    let x = 0;
    let y = 0;
    points.forEach((point, index) => {
      const binomial = factorial(n) / (factorial(index) * factorial(n - index));
      const factor = binomial * Math.pow(t, index) * Math.pow(1 - t, n - index);
      x += point.x * factor;
      y += point.y * factor;
    });
    result.push({ x, y });
  }
  return result;
}

function getPincerArrowHead(points) {
  const wholeLength = points.slice(1).reduce((length, point, index) => length + pointDistance(points[index], point), 0);
  const baseLength = Math.pow(wholeLength, 0.99);
  const tip = points[points.length - 1];
  const previous = points[points.length - 2];
  const segmentLength = pointDistance(previous, tip);
  const headHeight = Math.min(baseLength * 0.25, segmentLength);
  const headWidth = headHeight * 0.3;
  const neckHeight = headHeight * 0.85;
  const neckWidth = headHeight * 0.15;
  const headEnd = pointOffset(previous, tip, 0, headHeight, true);
  const neckEnd = pointOffset(previous, tip, 0, neckHeight, true);
  return [
    pointOffset(tip, neckEnd, Math.PI / 2, neckWidth, false),
    pointOffset(tip, headEnd, Math.PI / 2, headWidth, false),
    tip,
    pointOffset(tip, headEnd, Math.PI / 2, headWidth, true),
    pointOffset(tip, neckEnd, Math.PI / 2, neckWidth, true),
  ];
}

function getPincerArrowBody(points, neckLeft, neckRight, tailWidthFactor) {
  const wholeLength = points.slice(1).reduce((length, point, index) => length + pointDistance(points[index], point), 0);
  const baseLength = Math.pow(wholeLength, 0.99);
  const tailWidth = baseLength * tailWidthFactor;
  const neckWidth = pointDistance(neckLeft, neckRight);
  const widthDifference = (tailWidth - neckWidth) / 2;
  let travelled = 0;
  const left = [];
  const right = [];
  for (let index = 1; index < points.length - 1; index += 1) {
    const angle = pointAngle(points[index - 1], points[index], points[index + 1]) / 2;
    travelled += pointDistance(points[index - 1], points[index]);
    const width = (tailWidth / 2 - travelled / wholeLength * widthDifference) / Math.max(Math.sin(angle), 0.001);
    left.push(pointOffset(points[index - 1], points[index], Math.PI - angle, width, true));
    right.push(pointOffset(points[index - 1], points[index], angle, width, false));
  }
  return left.concat(right);
}

function getPincerArrowPoints(first, second, tip, clockwise) {
  const middle = pointMid(first, second);
  const length = pointDistance(middle, tip);
  let middle1 = pointOffset(tip, middle, 0, length * 0.3, true);
  let middle2 = pointOffset(tip, middle, 0, length * 0.5, true);
  middle1 = pointOffset(middle, middle1, Math.PI / 2, length / 5, clockwise);
  middle2 = pointOffset(middle, middle2, Math.PI / 2, length / 4, clockwise);
  const centerLine = [middle, middle1, middle2, tip];
  const arrowHead = getPincerArrowHead(centerLine);
  const bodyPoints = getPincerArrowBody(
    centerLine,
    arrowHead[0],
    arrowHead[4],
    pointDistance(first, second) / Math.pow(centerLine.slice(1).reduce((total, point, index) => total + pointDistance(centerLine[index], point), 0), 0.99) / 2,
  );
  const half = bodyPoints.length / 2;
  const left = bodyPoints.slice(0, half);
  const right = bodyPoints.slice(half);
  left.push(arrowHead[0]);
  right.push(arrowHead[4]);
  left.reverse();
  left.push(second);
  right.reverse();
  right.push(first);
  return left.reverse().concat(arrowHead, right);
}

function createReferencePincerPolygon(leftBase, rightBase, leftTip, rightTip) {
  const origin = midpoint(leftBase, rightBase);
  const frame = Cesium.Transforms.eastNorthUpToFixedFrame(origin);
  const inverseFrame = Cesium.Matrix4.inverseTransformation(frame, new Cesium.Matrix4());
  const toPoint = (position) => {
    const local = Cesium.Matrix4.multiplyByPoint(inverseFrame, position, new Cesium.Cartesian3());
    return { x: local.x, y: local.y };
  };
  const first = toPoint(leftBase);
  const second = toPoint(rightBase);
  const third = toPoint(leftTip);
  const fourth = toPoint(rightTip);
  const connection = pointMid(first, second);
  const clockwise = isClockwise(first, second, third);
  const leftArrow = clockwise
    ? getPincerArrowPoints(first, connection, fourth, false)
    : getPincerArrowPoints(second, connection, third, false);
  const rightArrow = clockwise
    ? getPincerArrowPoints(connection, second, third, true)
    : getPincerArrowPoints(connection, first, fourth, true);
  const arrowPointCount = leftArrow.length;
  const arrowHeadStart = (arrowPointCount - 5) / 2;
  const leftBodyStart = leftArrow.slice(0, arrowHeadStart);
  const leftHead = leftArrow.slice(arrowHeadStart, arrowHeadStart + 5);
  const leftBodyEnd = leftArrow.slice(arrowHeadStart + 5);
  const rightBodyStart = rightArrow.slice(0, arrowHeadStart);
  const rightHead = rightArrow.slice(arrowHeadStart, arrowHeadStart + 5);
  const rightBodyEnd = rightArrow.slice(arrowHeadStart + 5);
  const outline = [
    ...bezierPoints(rightBodyStart),
    ...rightHead,
    ...bezierPoints(rightBodyEnd.concat(leftBodyStart.slice(1))),
    ...leftHead,
    ...bezierPoints(leftBodyEnd),
  ];
  return outline.map((point) => {
    const world = Cesium.Matrix4.multiplyByPoint(frame, new Cesium.Cartesian3(point.x, point.y, 0), new Cesium.Cartesian3());
    return toSurface(world);
  });
}

function getArrowPolygons(type, positions) {
  if (positions.length < 2) return [];
  if (type === PLOTTING_TYPES.ATTACK_ARROW) {
    return positions.length >= 3
      ? [createCurvedAttackArrowPolygon(positions[0], positions[1], positions[2])]
      : [];
  }
  if (type === PLOTTING_TYPES.PINCER_ARROW) {
    if (positions.length < 3) return [];
    const rightTip = positions[3] || reflectPincerTip(positions[0], positions[1], positions[2]);
    return [createReferencePincerPolygon(positions[0], positions[1], positions[2], rightTip)];
  }
  return [];
}

function getArrowColor(type) {
  return type === PLOTTING_TYPES.PINCER_ARROW ? PINCER_COLOR : LINE_COLOR;
}

function getRectangle(positions) {
  return positions.length >= 2 ? Cesium.Rectangle.fromCartesianArray(positions) : undefined;
}

function appendOutlinedPolygon(entities, options) {
  const { positions, id, name, preview = false } = options;
  if (positions.length < 3) return;
  const polygon = options.collection.add({
    id: preview ? undefined : id,
    name,
    properties: preview ? undefined : { isLineAreaPlotting: true, plottingType: options.type },
    polygon: {
      hierarchy: options.hierarchy || new Cesium.PolygonHierarchy(positions),
      material: AREA_COLOR.withAlpha(preview ? 0.18 : 0.3),
      perPositionHeight: false,
    },
    polyline: {
      positions: options.outline || [...positions, positions[0]],
      width: 3,
      clampToGround: true,
      material: AREA_COLOR,
    },
  });
  entities.push(polygon);
}

export function createLineAreaPlottingController(viewer, onStateChange) {
  const dataSource = new Cesium.CustomDataSource("line-area-plotting");
  viewer.dataSources.add(dataSource);

  const state = {
    status: "idle",
    drawType: null,
    positions: [],
    previewPosition: null,
    drawings: [],
    nextId: 1,
    previewEntities: [],
    vertexEntities: [],
    freehandActive: false,
    previousRotateEnabled: null,
  };
  const handler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);

  function getActiveScene() {
    try {
      if (!viewer || viewer.isDestroyed?.()) return null;
      return viewer.scene || null;
    } catch {
      // React may unmount this controller after the parent Viewer has released its scene.
      return null;
    }
  }

  function snapshot() {
    return {
      status: state.status,
      drawType: state.drawType,
      vertexCount: state.positions.length,
      count: state.drawings.length,
    };
  }

  function notify() {
    onStateChange?.(snapshot());
  }

  function currentPositions() {
    return state.previewPosition ? [...state.positions, state.previewPosition] : state.positions;
  }

  function clearTemporaryEntities() {
    state.previewEntities.forEach((entity) => dataSource.entities.remove(entity));
    state.vertexEntities.forEach((entity) => dataSource.entities.remove(entity));
    state.previewEntities = [];
    state.vertexEntities = [];
  }

  function resetDrawing() {
    clearTemporaryEntities();
    restoreCameraRotate();
    state.status = "idle";
    state.drawType = null;
    state.positions = [];
    state.previewPosition = null;
    state.freehandActive = false;
    const scene = getActiveScene();
    if (scene?.canvas) scene.canvas.style.cursor = "default";
  }

  function disableCameraRotate() {
    if (state.previousRotateEnabled !== null) return;
    const controller = getActiveScene()?.screenSpaceCameraController;
    if (!controller) return;
    state.previousRotateEnabled = controller.enableRotate;
    controller.enableRotate = false;
  }

  function restoreCameraRotate() {
    if (state.previousRotateEnabled === null) return;
    const controller = getActiveScene()?.screenSpaceCameraController;
    if (controller) controller.enableRotate = state.previousRotateEnabled;
    state.previousRotateEnabled = null;
  }

  function addPreviewEntity(entity) {
    state.previewEntities.push(dataSource.entities.add(entity));
  }

  function createPreviewEntities() {
    const meta = TYPE_META[state.drawType];
    if (meta.category === "line") {
      addPreviewEntity({
        name: `${meta.name} Preview`,
        polyline: {
          positions: new Cesium.CallbackProperty(() => getLinePositions(state.drawType, currentPositions()), false),
          width: 3,
          clampToGround: true,
          material: new Cesium.PolylineGlowMaterialProperty({ color: LINE_COLOR, glowPower: 0.16 }),
        },
      });
      return;
    }

    if (meta.category === "rectangle") {
      addPreviewEntity({
        name: `${meta.name} Preview`,
        rectangle: {
          coordinates: new Cesium.CallbackProperty(() => getRectangle(currentPositions()), false),
          material: AREA_COLOR.withAlpha(0.18),
          outline: true,
          outlineColor: AREA_COLOR,
        },
      });
      return;
    }

    if (meta.category === "area") {
      addPreviewEntity({
        name: `${meta.name} Preview`,
        polygon: {
          hierarchy: new Cesium.CallbackProperty(() => {
            const positions = getAreaPositions(state.drawType, currentPositions());
            return positions.length >= 3 ? new Cesium.PolygonHierarchy(positions) : undefined;
          }, false),
          material: AREA_COLOR.withAlpha(0.18),
          perPositionHeight: false,
        },
        polyline: {
          positions: new Cesium.CallbackProperty(() => {
            const positions = getAreaPositions(state.drawType, currentPositions());
            return positions.length > 1 ? [...positions, positions[0]] : positions;
          }, false),
          width: 3,
          clampToGround: true,
          material: AREA_COLOR,
        },
      });
      return;
    }

    const arrowCount = 1;
    for (let index = 0; index < arrowCount; index += 1) {
      addPreviewEntity({
        name: `${meta.name} Preview`,
        polygon: {
          hierarchy: new Cesium.CallbackProperty(() => {
            const positions = getArrowPolygons(state.drawType, currentPositions())[index] || [];
            return positions.length >= 3 ? new Cesium.PolygonHierarchy(positions) : undefined;
          }, false),
          material: getArrowColor(state.drawType).withAlpha(0.26),
          perPositionHeight: false,
        },
      });
    }
  }

  function addVertex(position) {
    state.previewPosition = null;
    state.positions.push(position);
    state.vertexEntities.push(dataSource.entities.add({
      position,
      point: {
        pixelSize: 8,
        color: Cesium.Color.WHITE,
        outlineColor: TYPE_META[state.drawType].category === "area" ? AREA_COLOR : LINE_COLOR,
        outlineWidth: 2,
        heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
      },
    }));
  }

  function addPersistentEntities(type, positions, id) {
    const meta = TYPE_META[type];
    const entities = [];
    if (meta.category === "line") {
      entities.push(dataSource.entities.add({
        id,
        name: meta.name,
        properties: { isLineAreaPlotting: true, plottingType: type },
        polyline: {
          positions: getLinePositions(type, positions),
          width: 4,
          clampToGround: true,
          material: new Cesium.PolylineGlowMaterialProperty({ color: LINE_COLOR, glowPower: 0.18 }),
        },
      }));
    } else if (meta.category === "rectangle") {
      entities.push(dataSource.entities.add({
        id,
        name: meta.name,
        properties: { isLineAreaPlotting: true, plottingType: type },
        rectangle: {
          coordinates: getRectangle(positions),
          material: AREA_COLOR.withAlpha(0.3),
          outline: true,
          outlineColor: AREA_COLOR,
        },
      }));
    } else if (meta.category === "area") {
      appendOutlinedPolygon(entities, {
        collection: dataSource.entities,
        id,
        name: meta.name,
        type,
        positions: getAreaPositions(type, positions),
      });
    } else {
      const arrowColor = getArrowColor(type);
      getArrowPolygons(type, positions).forEach((arrowPositions, index) => {
        if (arrowPositions.length < 3) return;
        entities.push(dataSource.entities.add({
          id: index === 0 ? id : `${id}-${index + 1}`,
          name: meta.name,
          properties: { isLineAreaPlotting: true, plottingType: type },
          polygon: {
            hierarchy: new Cesium.PolygonHierarchy(arrowPositions),
            material: arrowColor.withAlpha(0.44),
            perPositionHeight: false,
          },
          polyline: {
            positions: [...arrowPositions, arrowPositions[0]],
            width: 3,
            clampToGround: true,
            material: arrowColor,
          },
        }));
      });
    }
    return entities;
  }

  function start(drawType) {
    if (!TYPE_META[drawType]) return;
    resetDrawing();
    const scene = getActiveScene();
    if (!scene?.canvas) return;
    state.status = "drawing";
    state.drawType = drawType;
    scene.canvas.style.cursor = "crosshair";
    createPreviewEntities();
    notify();
  }

  function cancel() {
    if (state.status !== "drawing") return;
    resetDrawing();
    notify();
  }

  function removeLastVertex() {
    if (state.status !== "drawing" || !state.positions.length) return;
    state.positions.pop();
    const vertex = state.vertexEntities.pop();
    if (vertex) dataSource.entities.remove(vertex);
    notify();
  }

  function complete() {
    if (state.status !== "drawing") return false;
    const meta = TYPE_META[state.drawType];
    if (state.positions.length < meta.minimumPoints) return false;
    const id = `plotting-${state.drawType}-${state.nextId++}`;
    const entities = addPersistentEntities(state.drawType, state.positions.slice(), id);
    if (!entities.length) return false;
    state.drawings.push({ id, entities });
    resetDrawing();
    notify();
    return true;
  }

  function clearAll() {
    resetDrawing();
    state.drawings.forEach(({ entities }) => entities.forEach((entity) => dataSource.entities.remove(entity)));
    state.drawings = [];
    notify();
  }

  function addFreehandPosition(position) {
    const previous = state.positions[state.positions.length - 1];
    if (!previous || Cesium.Cartesian3.distance(previous, position) > 45) addVertex(position);
  }

  handler.setInputAction((click) => {
    if (state.status !== "drawing" || TYPE_META[state.drawType].freehand) return;
    const position = getGroundPosition(viewer, click.position);
    if (!position) return;
    addVertex(position);
    if (TYPE_META[state.drawType].maximumPoints === state.positions.length) {
      complete();
    } else {
      notify();
    }
  }, Cesium.ScreenSpaceEventType.LEFT_CLICK);

  handler.setInputAction((click) => {
    if (state.status !== "drawing" || !TYPE_META[state.drawType].freehand) return;
    const position = getGroundPosition(viewer, click.position);
    if (!position) return;
    state.freehandActive = true;
    disableCameraRotate();
    addFreehandPosition(position);
    notify();
  }, Cesium.ScreenSpaceEventType.LEFT_DOWN);

  handler.setInputAction((movement) => {
    if (state.status !== "drawing") return;
    const position = getGroundPosition(viewer, movement.endPosition);
    if (!position) return;
    state.previewPosition = position;
    if (TYPE_META[state.drawType].freehand && state.freehandActive) {
      addFreehandPosition(position);
      notify();
    }
  }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);

  handler.setInputAction((click) => {
    if (state.status !== "drawing" || !TYPE_META[state.drawType].freehand || !state.freehandActive) return;
    const position = getGroundPosition(viewer, click.position);
    if (position) addFreehandPosition(position);
    state.freehandActive = false;
    restoreCameraRotate();
    if (!complete()) cancel();
  }, Cesium.ScreenSpaceEventType.LEFT_UP);

  handler.setInputAction(() => {
    if (state.status !== "drawing") return;
    if (!complete()) cancel();
  }, Cesium.ScreenSpaceEventType.RIGHT_CLICK);

  const keydownHandler = (event) => {
    if (state.status !== "drawing") return;
    if (event.key === "Escape") {
      event.preventDefault();
      cancel();
    } else if (event.key === "Enter") {
      event.preventDefault();
      complete();
    } else if (event.key === "Backspace") {
      event.preventDefault();
      removeLastVertex();
    }
  };
  window.addEventListener("keydown", keydownHandler);

  return {
    start,
    cancel,
    complete,
    removeLastVertex,
    clearAll,
    getSnapshot: snapshot,
    destroy() {
      handler.destroy();
      window.removeEventListener("keydown", keydownHandler);
      resetDrawing();
      if (getActiveScene()) viewer.dataSources.remove(dataSource, true);
    },
  };
}
