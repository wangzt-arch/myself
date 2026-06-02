import React, { useCallback } from "react";

function CityPopup({ city, onClose, onDrag }) {
  const handleMouseDown = useCallback((e) => {
    if (e.target.closest(".popup-close")) return;
    e.preventDefault();
    e.stopPropagation();

    let lastX = e.clientX;
    let lastY = e.clientY;

    const onMove = (ev) => {
      ev.preventDefault();
      onDrag(ev.clientX - lastX, ev.clientY - lastY);
      lastX = ev.clientX;
      lastY = ev.clientY;
    };

    const onUp = () => {
      document.removeEventListener("mousemove", onMove);
      document.removeEventListener("mouseup", onUp);
    };

    document.addEventListener("mousemove", onMove);
    document.addEventListener("mouseup", onUp);
  }, [onDrag]);

  return (
    <div className="city-popup">
      <button className="popup-close" onClick={(e) => { e.stopPropagation(); onClose(); }}>✕</button>
      <div className="popup-header" onMouseDown={handleMouseDown}>
        <div className="popup-city-dot" style={{ background: city.color }} />
        <h3>{city.name}</h3>
      </div>
      <p className="popup-desc">{city.description}</p>
      <div className="popup-stats">
        <div className="popup-stat">
          <span className="stat-label">人口</span>
          <span className="stat-value">{city.population}</span>
        </div>
        <div className="popup-stat">
          <span className="stat-label">面积</span>
          <span className="stat-value">{city.area}</span>
        </div>
      </div>
      <div className="popup-landmarks">
        <h4>特色建筑</h4>
        <ul>
          {city.landmarks.map((lm, i) => (
            <li key={i}>
              <span className="landmark-name">{lm.name}</span>
              <span className="landmark-desc">{lm.desc}</span>
            </li>
          ))}
        </ul>
      </div>
    </div>
  );
}

export default CityPopup;
