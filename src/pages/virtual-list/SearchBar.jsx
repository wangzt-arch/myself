import React from "react";
import "./SearchBar.scss";

function SearchBar({ value, onChange }) {
  return (
    <div className={`sb ${value ? "has-value" : ""}`}>
      <div className="sb-icon">
        <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" strokeWidth="1.8">
          <circle cx="11" cy="11" r="6.5" />
          <path d="M16 16l5 5" strokeLinecap="round" />
        </svg>
      </div>
      <input
        type="text"
        className="sb-input"
        placeholder="搜索文章、作者、分类..."
        value={value}
        onChange={(e) => onChange(e.target.value)}
      />
      {value && (
        <button className="sb-clear" onClick={() => onChange("")} aria-label="清除">
          <svg viewBox="0 0 24 24" width="12" height="12" fill="none" stroke="currentColor" strokeWidth="2.2">
            <path d="M5 5l14 14M19 5L5 19" strokeLinecap="round" />
          </svg>
        </button>
      )}
      <div className="sb-glow" />
    </div>
  );
}

export default SearchBar;
