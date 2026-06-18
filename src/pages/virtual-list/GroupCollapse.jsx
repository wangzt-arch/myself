import React, { useState } from "react";
import "./GroupCollapse.scss";

function GroupCollapse({ title, count, items, selectedIds, onToggleSelect, accentColor = "#ff7a45", accentBg = "rgba(255,122,69,0.12)", accentLabel }) {
  const [isExpanded, setIsExpanded] = useState(true);
  const selectedCount = items.filter((item) => selectedIds.has(item.id)).length;

  return (
    <div className={`gc ${isExpanded ? "expanded" : "collapsed"}`} style={{ "--gc-accent": accentColor, "--gc-bg": accentBg }}>
      <div className="gc-header" onClick={() => setIsExpanded(!isExpanded)}>
        <div className="gc-left">
          <span className="gc-toggle">
            <svg viewBox="0 0 24 24" width="10" height="10" fill="none" stroke="currentColor" strokeWidth="2.5">
              <path d="M6 9l6 6 6-6" strokeLinecap="round" strokeLinejoin="round" />
            </svg>
          </span>
          <span className="gc-badge">{accentLabel}</span>
          <span className="gc-title">{title}</span>
        </div>
        <div className="gc-right">
          {selectedCount > 0 && <span className="gc-selected">已选 {selectedCount}</span>}
          <span className="gc-count">{count}</span>
        </div>
      </div>

      {isExpanded && (
        <div className="gc-body">
          {items.map((item) => {
            const isSelected = selectedIds.has(item.id);
            return (
              <div
                key={item.id}
                className={`gc-item ${isSelected ? "checked" : ""}`}
                onClick={() => onToggleSelect(item.id)}
              >
                <div className="gc-checkbox">
                  <div className="gc-check-inner">
                    {isSelected && (
                      <svg viewBox="0 0 24 24" width="10" height="10" fill="none" stroke="currentColor" strokeWidth="3.5">
                        <path d="M4 12l5 5L20 6" strokeLinecap="round" strokeLinejoin="round" />
                      </svg>
                    )}
                  </div>
                </div>
                <div className="gc-main">
                  <div className="gc-item-title">{item.title}</div>
                  <div className="gc-item-meta">
                    <span>{item.author}</span>
                    <span className="gc-sep">·</span>
                    <span>{item.date}</span>
                  </div>
                </div>
                <div className="gc-item-stats">
                  <span>👁 {item.views}</span>
                </div>
              </div>
            );
          })}
        </div>
      )}
    </div>
  );
}

export default GroupCollapse;
