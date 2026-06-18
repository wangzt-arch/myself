import React from "react";
import "./Skeleton.scss";

function Skeleton({ count = 5 }) {
  return (
    <div className="sk-list">
      {Array.from({ length: count }).map((_, i) => (
        <div key={i} className="sk-item" style={{ animationDelay: `${i * 50}ms` }}>
          <div className="sk-accent shimmer" />
          <div className="sk-index shimmer" />
          <div className="sk-avatar shimmer" />
          <div className="sk-body">
            <div className="sk-title shimmer" />
            <div className="sk-meta-row">
              <div className="sk-meta-1 shimmer" />
              <div className="sk-meta-2 shimmer" />
              <div className="sk-meta-3 shimmer" />
              <div className="sk-spacer" />
              <div className="sk-meta-4 shimmer" />
            </div>
          </div>
        </div>
      ))}
    </div>
  );
}

export default Skeleton;
