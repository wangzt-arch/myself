import symbol1 from './military-symbols/1.svg';
import symbol2 from './military-symbols/2.svg';
import symbol3 from './military-symbols/3.svg';
import symbol4 from './military-symbols/4.svg';
import symbol5 from './military-symbols/5.svg';
import symbol6 from './military-symbols/6.svg';
import symbol7 from './military-symbols/7.svg';
import symbol8 from './military-symbols/8.svg';
import symbol9 from './military-symbols/9.svg';
import symbol10 from './military-symbols/10.svg';
import symbol11 from './military-symbols/11.svg';
import symbol12 from './military-symbols/12.svg';
import symbol13 from './military-symbols/13.svg';
import symbol14 from './military-symbols/14.svg';
import symbol15 from './military-symbols/15.svg';
import symbol16 from './military-symbols/16.svg';
import symbol17 from './military-symbols/17.svg';
import symbol18 from './military-symbols/18.svg';
import symbol19 from './military-symbols/19.svg';
import symbol20 from './military-symbols/20.svg';
import symbol21 from './military-symbols/21.svg';
import symbol22 from './military-symbols/22.svg';
import symbol23 from './military-symbols/23.svg';
import symbol24 from './military-symbols/24.svg';
import symbol25 from './military-symbols/25.svg';
import symbol26 from './military-symbols/26.svg';
import symbol27 from './military-symbols/27.svg';
import symbol28 from './military-symbols/28.svg';
import symbol29 from './military-symbols/29.svg';
import symbol30 from './military-symbols/30.svg';
import symbol31 from './military-symbols/31.svg';
import symbol32 from './military-symbols/32.svg';
import symbol33 from './military-symbols/33.svg';
import symbol34 from './military-symbols/34.svg';
import symbol35 from './military-symbols/35.svg';
import symbolHjjd from './military-symbols/hjjd.svg';
import symbolExport from './military-symbols/导出.svg';
import * as Cesium from "cesium";


export const MILITARY_SYMBOLS = [
  { id: '1', label: '军标1', icon: symbol1 },
  { id: '2', label: '军标2', icon: symbol2 },
  { id: '3', label: '军标3', icon: symbol3 },
  { id: '4', label: '军标4', icon: symbol4 },
  { id: '5', label: '军标5', icon: symbol5 },
  { id: '6', label: '军标6', icon: symbol6 },
  { id: '7', label: '军标7', icon: symbol7 },
  { id: '8', label: '军标8', icon: symbol8 },
  { id: '9', label: '军标9', icon: symbol9 },
  { id: '10', label: '军标10', icon: symbol10 },
  { id: '11', label: '军标11', icon: symbol11 },
  { id: '12', label: '军标12', icon: symbol12 },
  { id: '13', label: '军标13', icon: symbol13 },
  { id: '14', label: '军标14', icon: symbol14 },
  { id: '15', label: '军标15', icon: symbol15 },
  { id: '16', label: '军标16', icon: symbol16 },
  { id: '17', label: '军标17', icon: symbol17 },
  { id: '18', label: '军标18', icon: symbol18 },
  { id: '19', label: '军标19', icon: symbol19 },
  { id: '20', label: '军标20', icon: symbol20 },
  { id: '21', label: '军标21', icon: symbol21 },
  { id: '22', label: '军标22', icon: symbol22 },
  { id: '23', label: '军标23', icon: symbol23 },
  { id: '24', label: '军标24', icon: symbol24 },
  { id: '25', label: '军标25', icon: symbol25 },
  { id: '26', label: '军标26', icon: symbol26 },
  { id: '27', label: '军标27', icon: symbol27 },
  { id: '28', label: '军标28', icon: symbol28 },
  { id: '29', label: '军标29', icon: symbol29 },
  { id: '30', label: '军标30', icon: symbol30 },
  { id: '31', label: '军标31', icon: symbol31 },
  { id: '32', label: '军标32', icon: symbol32 },
  { id: '33', label: '军标33', icon: symbol33 },
  { id: '34', label: '军标34', icon: symbol34 },
  { id: '35', label: '军标35', icon: symbol35 },
  { id: 'hjjd', label: '海军基地', icon: symbolHjjd },
  { id: 'export', label: '指挥所', icon: symbolExport },
];

export function createMilitarySymbolController(viewer, onStatusChange) {
  let activeSymbolId = null;
  let symbols = [];
  let nextId = 1;
  let isPlacing = false;

  const handler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);

  handler.setInputAction((click) => {
    if (!isPlacing || !activeSymbolId) return;

    const ray = viewer.camera.getPickRay(click.position);
    const position = ray
      ? viewer.scene.globe.pick(ray, viewer.scene)
      : viewer.camera.pickEllipsoid(click.position, viewer.scene.globe.ellipsoid);

    if (!position) return;

    const symbolData = MILITARY_SYMBOLS.find((s) => s.id === activeSymbolId);
    if (!symbolData) return;

    const entity = viewer.entities.add({
      id: `military-symbol-${nextId++}`,
      name: symbolData.label,
      position: position,
      billboard: {
        image: symbolData.icon,
        width: 48,
        height: 48,
        heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
        scaleByDistance: new Cesium.NearFarScalar(10000, 1.5, 5000000, 0.5),
        verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
      },
    });

    symbols.push({
      id: entity.id,
      symbolId: activeSymbolId,
      label: symbolData.label,
      entity,
    });

    if (onStatusChange) {
      onStatusChange(getSnapshot());
    }
  }, Cesium.ScreenSpaceEventType.LEFT_CLICK);

  // 右键停止添加军标
  handler.setInputAction(() => {
    if (isPlacing) {
      setActiveSymbol(null);
    }
  }, Cesium.ScreenSpaceEventType.RIGHT_CLICK);

  function setActiveSymbol(symbolId) {
    activeSymbolId = symbolId;
    isPlacing = symbolId != null;
    viewer.scene.canvas.style.cursor = isPlacing ? 'crosshair' : 'default';
    if (onStatusChange) {
      onStatusChange(getSnapshot());
    }
  }

  function getActiveSymbolId() {
    return activeSymbolId;
  }

  function getSymbols() {
    return symbols.map((s) => ({
      id: s.id,
      symbolId: s.symbolId,
      label: s.label,
    }));
  }

  function removeSymbol(id) {
    const index = symbols.findIndex((s) => s.id === id);
    if (index === -1) return;
    viewer.entities.remove(symbols[index].entity);
    symbols.splice(index, 1);
    if (onStatusChange) {
      onStatusChange(getSnapshot());
    }
  }

  function clearAll() {
    symbols.forEach((s) => viewer.entities.remove(s.entity));
    symbols = [];
    if (onStatusChange) {
      onStatusChange(getSnapshot());
    }
  }

  function getSnapshot() {
    return {
      activeSymbolId,
      isPlacing,
      count: symbols.length,
      symbols: getSymbols(),
    };
  }

  function destroy() {
    handler.destroy();
    clearAll();
    viewer.scene.canvas.style.cursor = 'default';
  }

  return {
    setActiveSymbol,
    getActiveSymbolId,
    getSymbols,
    removeSymbol,
    clearAll,
    getSnapshot,
    destroy,
  };
}
