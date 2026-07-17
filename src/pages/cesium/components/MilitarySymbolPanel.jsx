import { useCallback, useEffect, useRef, useState } from 'react';
import { createMilitarySymbolController, MILITARY_SYMBOLS } from '../militarySymbol';

/**
 * 军标标注面板组件
 * 在功能面板中渲染军标选择网格，点击军标后在地球上放置
 * 右键取消添加，支持清空所有军标
 */
function MilitarySymbolPanel({ viewerRef }) {
  const controllerRef = useRef(null);
  const [state, setState] = useState({
    activeSymbolId: null,
    isPlacing: false,
    count: 0,
    symbols: [],
  });

  // 初始化控制器
  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer) return;

    controllerRef.current = createMilitarySymbolController(viewer, (snapshot) => {
      setState(snapshot);
    });

    return () => {
      controllerRef.current?.destroy();
      controllerRef.current = null;
    };
  }, [viewerRef]);

  // 点击军标图标，选中/取消选中
  const handleSelectSymbol = useCallback((symbolId) => {
    const controller = controllerRef.current;
    if (!controller) return;
    if (controller.getActiveSymbolId() === symbolId) {
      controller.setActiveSymbol(null);
    } else {
      controller.setActiveSymbol(symbolId);
    }
  }, []);

  // 清空所有军标
  const handleClearAll = useCallback(() => {
    controllerRef.current?.clearAll();
  }, []);

  return (
    <section className="feature-section military-symbol-panel">
      <div className="military-symbol-panel__title">
        <div>
          <span>Military Symbols</span>
          <strong>军标标注</strong>
        </div>
        <i>已放置 {state.count} 个</i>
      </div>

      <div className="military-symbol-panel__hint">
        {state.isPlacing
          ? '点击地图放置军标，右键取消'
          : '选择军标后在地图上点击放置'}
      </div>

      <div className="military-symbol-panel__grid">
        {MILITARY_SYMBOLS.map((symbol) => (
          <button
            key={symbol.id}
            type="button"
            className={`military-symbol-panel__item ${state.activeSymbolId === symbol.id ? "military-symbol-panel__item--active" : ""}`}
            onClick={() => handleSelectSymbol(symbol.id)}
            title={symbol.label}
          >
            <img src={symbol.icon} alt={symbol.label} />
          </button>
        ))}
      </div>

      <div className="military-symbol-panel__actions">
        <button
          type="button"
          className="military-symbol-panel__clear-btn"
          onClick={handleClearAll}
          disabled={state.count === 0}
        >
          清空军标
        </button>
      </div>
    </section>
  );
}

export default MilitarySymbolPanel;
