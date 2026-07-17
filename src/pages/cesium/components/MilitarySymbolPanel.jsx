import { useCallback, useRef } from 'react';
import { MILITARY_SYMBOLS } from '../militarySymbol';

/**
 * 军标标注面板组件（纯 UI，不含右键菜单和编辑面板）
 */
function MilitarySymbolPanel({ controllerRef, state }) {
  const panelRef = useRef(null);

  const handleSelectSymbol = useCallback((symbolId) => {
    const controller = controllerRef.current;
    if (!controller) return;
    if (controller.getActiveSymbolId() === symbolId) {
      controller.setActiveSymbol(null);
    } else {
      controller.setActiveSymbol(symbolId);
    }
  }, [controllerRef]);

  const handleClearAll = useCallback(() => {
    controllerRef.current?.clearAll();
  }, [controllerRef]);

  return (
    <section className="feature-section military-symbol-panel" ref={panelRef}>
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
          : state.selectedSymbolId
            ? '已选中军标，右键打开菜单'
            : '选择军标后在地图上点击放置，左键拾取'}
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
