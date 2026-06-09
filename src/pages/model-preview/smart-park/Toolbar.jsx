import React from 'react';

function Toolbar({ onViewChange, currentView, weather, onWeatherChange, timeOfDay, onTimeChange }) {
  const views = [
    { id: 'overview', label: '🔭 全景', icon: '🔭' },
    { id: 'top', label: '⬇️ 俯视', icon: '⬇️' },
    { id: 'side', label: '👁️ 平视', icon: '👁️' },
    { id: 'tour', label: '🚁 漫游', icon: '🚁' },
  ];

  const weathers = [
    { id: 'sunny', label: '☀️ 晴', icon: '☀️' },
    { id: 'cloudy', label: '☁️ 多云', icon: '☁️' },
    { id: 'rainy', label: '🌧️ 雨', icon: '🌧️' },
    { id: 'night', label: '🌙 夜', icon: '🌙' },
  ];

  const times = [
    { id: 'morning', label: '🌅 晨', icon: '🌅' },
    { id: 'noon', label: '☀️ 午', icon: '☀️' },
    { id: 'evening', label: '🌇 暮', icon: '🌇' },
    { id: 'night', label: '🌃 夜', icon: '🌃' },
  ];

  return (
    <div className="smart-park__toolbar">
      {/* 视角切换 */}
      <div className="toolbar__group">
        <div className="toolbar__label">视角</div>
        <div className="toolbar__buttons">
          {views.map((view) => (
            <button
              key={view.id}
              className={`toolbar__btn ${currentView === view.id ? 'toolbar__btn--active' : ''}`}
              onClick={() => onViewChange(view.id)}
              title={view.label}
            >
              {view.icon}
            </button>
          ))}
        </div>
      </div>

      {/* 天气切换 */}
      <div className="toolbar__group">
        <div className="toolbar__label">天气</div>
        <div className="toolbar__buttons">
          {weathers.map((w) => (
            <button
              key={w.id}
              className={`toolbar__btn ${weather === w.id ? 'toolbar__btn--active' : ''}`}
              onClick={() => onWeatherChange(w.id)}
              title={w.label}
            >
              {w.icon}
            </button>
          ))}
        </div>
      </div>

      {/* 时间切换 */}
      <div className="toolbar__group">
        <div className="toolbar__label">时间</div>
        <div className="toolbar__buttons">
          {times.map((t) => (
            <button
              key={t.id}
              className={`toolbar__btn ${timeOfDay === t.id ? 'toolbar__btn--active' : ''}`}
              onClick={() => onTimeChange(t.id)}
              title={t.label}
            >
              {t.icon}
            </button>
          ))}
        </div>
      </div>
    </div>
  );
}

export default Toolbar;
