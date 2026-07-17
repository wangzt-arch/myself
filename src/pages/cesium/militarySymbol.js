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


// 解析颜色字符串：HTML input[type=color] 返回的 6 位 hex 不带 # 前缀，需要补上
function parseColor(colorStr, fallback) {
  if (!colorStr) {
    return Cesium.Color.fromCssColorString(fallback);
  }
  const str = colorStr.startsWith('#') ? colorStr : `#${colorStr}`;
  return Cesium.Color.fromCssColorString(str);
}

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

export function createMilitarySymbolController(viewer, onStatusChange, onContextMenu) {
  let activeSymbolId = null;
  let symbols = [];
  let nextId = 1;
  let isPlacing = false;
  let selectedSymbolId = null;

  const handler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);

  // 禁用 Cesium canvas 上的浏览器默认右键菜单，确保右键事件能被 Cesium 捕获
  viewer.scene.canvas.addEventListener('contextmenu', (e) => e.preventDefault());

  // 左键：放置军标 或 拾取已有军标
  handler.setInputAction((click) => {
    // 放置模式：点击地图放置军标
    if (isPlacing && activeSymbolId) {
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
          color: parseColor('#ffffff'),
        },
        label: {
          text: symbolData.label,
          font: '12px sans-serif',
          fillColor: Cesium.Color.WHITE,
          outlineColor: Cesium.Color.BLACK,
          outlineWidth: 2,
          style: Cesium.LabelStyle.FILL_AND_OUTLINE,
          verticalOrigin: Cesium.VerticalOrigin.TOP,
          pixelOffset: new Cesium.Cartesian2(0, 4),
          heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
          show: true,
        },
        properties: {
          isMilitarySymbol: true,
          symbolId: activeSymbolId,
          size: 48,
          opacity: 1,
          showLabel: true,
          labelText: symbolData.label,
          labelColor: '#ffffff',
          billboardColor: '#ffffff',
        },
      });

      // 计算经纬度高度
      const cartographic = Cesium.Cartographic.fromCartesian(position);
      const lon = Cesium.Math.toDegrees(cartographic.longitude);
      const lat = Cesium.Math.toDegrees(cartographic.latitude);
      const height = cartographic.height;

      symbols.push({
        id: entity.id,
        symbolId: activeSymbolId,
        label: symbolData.label,
        entity,
        lon,
        lat,
        height,
      });

      if (onStatusChange) {
        onStatusChange(getSnapshot());
      }
      return;
    }

    // 非放置模式：拾取军标
    const picked = viewer.scene.pick(click.position);
    if (Cesium.defined(picked) && Cesium.defined(picked.id)) {
      const symbolItem = symbols.find((s) => s.id === picked.id.id || s.entity === picked.id);
      if (symbolItem) {
        selectSymbol(symbolItem.id);
        return;
      }
    }

    // 点击空白处取消选中
    if (selectedSymbolId) {
      deselectSymbol();
      // 取消选中时关闭右键菜单和编辑面板
      if (onContextMenu) {
        onContextMenu(null);
      }
    }
  }, Cesium.ScreenSpaceEventType.LEFT_CLICK);

  // 右键：取消放置 或 弹出属性菜单
  handler.setInputAction((click) => {
    if (isPlacing) {
      setActiveSymbol(null);
      return;
    }

    // 右键点击军标，弹出属性菜单
    const picked = viewer.scene.pick(click.position);
    if (Cesium.defined(picked) && Cesium.defined(picked.id)) {
      const symbolItem = symbols.find((s) => s.id === picked.id.id || s.entity === picked.id);
      if (symbolItem) {
        selectSymbol(symbolItem.id);
        // 计算菜单位置：图标右侧
        const canvasRect = viewer.scene.canvas.getBoundingClientRect();
        let menuX = canvasRect.left + click.position.x;
        let menuY = canvasRect.top + click.position.y;

        // 尝试获取军标屏幕位置，菜单显示在图标右侧
        try {
          const position = symbolItem.entity.position.getValue(viewer.clock.currentTime);
          if (position) {
            const windowPos = Cesium.SceneTransforms.worldToWindowCoordinates(
              viewer.scene,
              position
            );
            if (windowPos) {
              menuX = canvasRect.left + windowPos.x + 50;
              menuY = canvasRect.top + windowPos.y - 20;
            }
          }
        } catch (e) {
          // 坐标转换失败，使用点击位置
          menuX = canvasRect.left + click.position.x + 20;
          menuY = canvasRect.top + click.position.y;
        }

        if (onContextMenu) {
          const props = getSymbolProperties(symbolItem.id);
          if (props) {
            onContextMenu({
              ...props,
              x: menuX,
              y: menuY,
            });
          }
        }
        return;
      }
    }
  }, Cesium.ScreenSpaceEventType.RIGHT_CLICK);

  // 选中军标：添加发光效果
  function selectSymbol(symbolId) {
    // 先取消之前的选中
    if (selectedSymbolId) {
      applyDeselectEffect(selectedSymbolId);
    }

    selectedSymbolId = symbolId;
    applySelectEffect(symbolId);

    if (onStatusChange) {
      onStatusChange(getSnapshot());
    }
  }

  function deselectSymbol() {
    if (selectedSymbolId) {
      applyDeselectEffect(selectedSymbolId);
      selectedSymbolId = null;
      if (onStatusChange) {
        onStatusChange(getSnapshot());
      }
    }
  }

  // 选中效果：仅放大
  function applySelectEffect(symbolId) {
    const item = symbols.find((s) => s.id === symbolId);
    if (!item || !item.entity) return;

    const billboard = item.entity.billboard;
    if (billboard) {
      billboard.scale = 1.2;
    }
  }

  // 取消选中：恢复原始状态
  function applyDeselectEffect(symbolId) {
    const item = symbols.find((s) => s.id === symbolId);
    if (!item || !item.entity) return;

    const billboard = item.entity.billboard;
    if (billboard) {
      billboard.scale = 1.0;
    }
  }

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
      entity: s.entity,
    }));
  }

  // 获取军标属性
  function getSymbolProperties(symbolId) {
    const item = symbols.find((s) => s.id === symbolId);
    if (!item || !item.entity) return null;

    const props = item.entity.properties;
    return {
      id: item.id,
      symbolId: props.symbolId.getValue(),
      label: props.labelText.getValue(),
      size: props.size.getValue(),
      opacity: props.opacity.getValue(),
      showLabel: props.showLabel.getValue(),
      labelColor: props.labelColor?.getValue() || '#ffffff',
      billboardColor: props.billboardColor?.getValue() || '#ffffff',
      lon: item.lon,
      lat: item.lat,
      height: item.height,
    };
  }

  // 更新军标属性
  function updateSymbolProperties(symbolId, updates) {
    const item = symbols.find((s) => s.id === symbolId);
    if (!item || !item.entity) return;

    const entity = item.entity;
    const props = entity.properties;

    if (updates.size !== undefined) {
      props.size = updates.size;
      entity.billboard.width = updates.size;
      entity.billboard.height = updates.size;
    }

    if (updates.opacity !== undefined) {
      props.opacity = updates.opacity;
      const color = parseColor(props.billboardColor?.getValue(), '#ffffff');
      entity.billboard.color = color.withAlpha(updates.opacity);
    }

    if (updates.showLabel !== undefined) {
      props.showLabel = updates.showLabel;
      entity.label.show = updates.showLabel;
    }

    if (updates.labelText !== undefined) {
      props.labelText = updates.labelText;
      entity.label.text = updates.labelText;
      item.label = updates.labelText;
    }

    // 名称颜色
    if (updates.labelColor !== undefined) {
      props.labelColor = updates.labelColor;
      entity.label.fillColor = parseColor(updates.labelColor, '#ffffff');
    }

    // 军标颜色
    if (updates.billboardColor !== undefined) {
      props.billboardColor = updates.billboardColor;
      const opacity = props.opacity.getValue();
      const color = parseColor(updates.billboardColor, '#ffffff');
      entity.billboard.color = color.withAlpha(opacity);
    }

    // 经纬度高度修改
    if (updates.lon !== undefined || updates.lat !== undefined || updates.height !== undefined) {
      const newLon = updates.lon !== undefined ? updates.lon : item.lon;
      const newLat = updates.lat !== undefined ? updates.lat : item.lat;
      const newHeight = updates.height !== undefined ? updates.height : item.height;

      const newPosition = Cesium.Cartesian3.fromDegrees(newLon, newLat, newHeight);
      entity.position = newPosition;

      item.lon = newLon;
      item.lat = newLat;
      item.height = newHeight;
    }

    if (onStatusChange) {
      onStatusChange(getSnapshot());
    }
  }

  // 定位到军标
  function flyToSymbol(symbolId) {
    const item = symbols.find((s) => s.id === symbolId);
    if (!item || !item.entity) return;

    const position = item.entity.position.getValue(viewer.clock.currentTime);
    if (!position) return;

    const cartographic = Cesium.Cartographic.fromCartesian(position);
    viewer.camera.flyTo({
      destination: Cesium.Cartesian3.fromDegrees(
        Cesium.Math.toDegrees(cartographic.longitude),
        Cesium.Math.toDegrees(cartographic.latitude),
        cartographic.height + 500000
      ),
      duration: 1.2,
    });
  }

  function removeSymbol(id) {
    const index = symbols.findIndex((s) => s.id === id);
    if (index === -1) return;

    // 如果删除的是选中的军标，先取消选中
    if (selectedSymbolId === id) {
      applyDeselectEffect(id);
      selectedSymbolId = null;
    }

    // 移除光环
    if (symbols[index]._glowEntity) {
      viewer.entities.remove(symbols[index]._glowEntity);
    }

    viewer.entities.remove(symbols[index].entity);
    symbols.splice(index, 1);
    if (onStatusChange) {
      onStatusChange(getSnapshot());
    }
  }

  function clearAll() {
    symbols.forEach((s) => {
      if (s._glowEntity) viewer.entities.remove(s._glowEntity);
      viewer.entities.remove(s.entity);
    });
    symbols = [];
    selectedSymbolId = null;
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
      selectedSymbolId,
      contextMenu: null,
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
    selectSymbol,
    deselectSymbol,
    getSymbolProperties,
    updateSymbolProperties,
    flyToSymbol,
  };
}
