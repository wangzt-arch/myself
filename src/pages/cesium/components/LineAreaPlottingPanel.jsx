import { useCallback, useEffect, useRef, useState } from "react";
import {
  PLOTTING_TOOL_GROUPS,
  createLineAreaPlottingController,
  getPlottingMinimumPoints,
} from "../lineAreaPlotting";
import "./LineAreaPlottingPanel.scss";

const INITIAL_STATE = {
  status: "idle",
  drawType: null,
  vertexCount: 0,
  count: 0,
};

function LineAreaPlottingPanel({ viewerRef }) {
  const controllerRef = useRef(null);
  const [isOpen, setIsOpen] = useState(false);
  const [plottingState, setPlottingState] = useState(INITIAL_STATE);

  useEffect(() => {
    let animationFrameId;
    let disposed = false;

    const initialize = () => {
      if (disposed) return;
      const viewer = viewerRef.current;
      if (!viewer || viewer.isDestroyed?.()) {
        animationFrameId = window.requestAnimationFrame(initialize);
        return;
      }

      const controller = createLineAreaPlottingController(viewer, setPlottingState);
      controllerRef.current = controller;
      setPlottingState(controller.getSnapshot());
    };

    initialize();

    return () => {
      disposed = true;
      if (animationFrameId) window.cancelAnimationFrame(animationFrameId);
      controllerRef?.current?.destroy();
      controllerRef.current = null;
    };
  }, [viewerRef]);

  const startDrawing = useCallback((type) => {
    const controller = controllerRef.current;
    if (!controller) return;
    if (plottingState.status === "drawing" && plottingState.drawType === type) {
      controller.cancel();
      return;
    }
    controller.start(type);
  }, [plottingState.drawType, plottingState.status]);

  const isDrawing = plottingState.status === "drawing";
  const canComplete = isDrawing && plottingState.vertexCount >= getPlottingMinimumPoints(plottingState.drawType);

  return (
    <aside className={isOpen ? "line-area-plotting" : "line-area-plotting line-area-plotting--collapsed"}>
      <div className="line-area-plotting__panel-header">
        <div>
          <span>Tactical Plotting</span>
          <strong>线面标绘</strong>
        </div>
        <button
          className="line-area-plotting__toggle"
          type="button"
          onClick={() => setIsOpen((open) => !open)}
          aria-expanded={isOpen}
          aria-label={isOpen ? "收起线面标绘面板" : "展开线面标绘面板"}
        >
          {isOpen ? "收起" : "展开"}
        </button>
      </div>

      {isOpen && (
        <div className="line-area-plotting__body">
          <div className="line-area-plotting__header">
            <span>当前标绘</span>
            <i>{isDrawing ? `${plottingState.vertexCount} 点` : `${plottingState.count} 个图形`}</i>
          </div>

          {PLOTTING_TOOL_GROUPS.map((group) => (
            <section className="line-area-plotting__group" key={group.label}>
              <span>{group.label}</span>
              <div className="line-area-plotting__tools" aria-label={group.label}>
                {group.tools.map(([type, label]) => (
                  <button
                    className={isDrawing && plottingState.drawType === type ? "line-area-plotting__tool line-area-plotting__tool--active" : "line-area-plotting__tool"}
                    key={type}
                    type="button"
                    onClick={() => startDrawing(type)}
                  >
                    {label}
                  </button>
                ))}
              </div>
            </section>
          ))}

          <div className="line-area-plotting__actions">
            <button type="button" onClick={() => controllerRef.current?.removeLastVertex()} disabled={!isDrawing || !plottingState.vertexCount}>
              撤销点
            </button>
            <button type="button" onClick={() => controllerRef.current?.cancel()} disabled={!isDrawing}>
              取消
            </button>
            <button className="line-area-plotting__finish" type="button" onClick={() => controllerRef.current?.complete()} disabled={!canComplete}>
              完成
            </button>
          </div>

          <button
            className="line-area-plotting__clear"
            type="button"
            onClick={() => controllerRef.current?.clearAll()}
            disabled={!plottingState.count && !isDrawing}
          >
            清空线面标绘
          </button>
        </div>
      )}
    </aside>
  );
}

export default LineAreaPlottingPanel;
