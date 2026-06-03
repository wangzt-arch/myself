// Cesium Ion Token
export const CESIUM_TOKEN =
  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiJlZDA2OTcyOC01MTU5LTQ0MWQtODExMi0zYTZjNzE5NWYwMGYiLCJpZCI6NDM4OTQyLCJpc3MiOiJodHRwczovL2FwaS5jZXNpdW0uY29tIiwiYXVkIjoidW5kZWZpbmVkX2RlZmF1bHQiLCJpYXQiOjE3ODAzMTAyMDd9.7H3KXRDhmnCOaRVGVmzRs_snRWKTWQbhEN1sQbVbQQA";

// Viewer 初始化配置
export const VIEWER_OPTIONS = {
  baseLayerPicker: false,
  geocoder: false,
  homeButton: true,
  sceneModePicker: true,
  navigationHelpButton: false,
  animation: true,
  timeline: true,
  fullscreenButton: true,
  vrButton: false,
  infoBox: false,
  selectionIndicator: false,
  targetFrameRate: 60,
};

// 初始视角
export const INITIAL_CAMERA = {
  destination: { lon: 105, lat: 35, height: 8000000 },
  orientation: { heading: 0, pitch: -90, roll: 0 },
  duration: 3,
};

// 面板统计数据
export const STATS = {
  provinces: 34,
  cities: 6,
  satellites: 1,
  status: "ONLINE",
};
