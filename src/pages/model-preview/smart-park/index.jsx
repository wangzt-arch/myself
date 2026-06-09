import React, { useState, useCallback } from 'react';
import { Canvas } from '@react-three/fiber';
import { Scene, PARK_DATA } from './Scene';
import DataPanel from './DataPanel';
import Toolbar from './Toolbar';

function BuildingDetailPopup({ building, onClose }) {
  if (!building) return null;

  return (
    <div className="building-popup">
      <div className="building-popup__header">
        <span className="building-popup__title">🔍 建筑详情</span>
        <button className="building-popup__close" onClick={onClose}>✕</button>
      </div>
      <div className="building-popup__body">
        <div className="popup__name">{building.name}</div>
        <div className="building-popup__divider" />
        <div className="building-detail">
          <div className="detail-row">
            <span className="detail-label">类型</span>
            <span className="detail-value">{building.type}</span>
          </div>
          <div className="detail-row">
            <span className="detail-label">楼层</span>
            <span className="detail-value">{building.floors}层</span>
          </div>
          <div className="detail-row">
            <span className="detail-label">面积</span>
            <span className="detail-value">
              {Math.floor(building.size[0] * building.size[2] * building.floors * 50)}㎡
            </span>
          </div>
          <div className="detail-row">
            <span className="detail-label">长×宽×高</span>
            <span className="detail-value">
              {building.size[0].toFixed(1)}×{building.size[2].toFixed(1)}×{building.size[1].toFixed(1)}m
            </span>
          </div>
          <div className="detail-row">
            <span className="detail-label">设备数</span>
            <span className="detail-value">{Math.floor(building.floors * 12 + 20)}台</span>
          </div>
          <div className="detail-row">
            <span className="detail-label">今日能耗</span>
            <span className="detail-value">{Math.floor(building.floors * 85 + 120)}kWh</span>
          </div>
          <div className="detail-row">
            <span className="detail-label">状态</span>
            <span className="detail-value detail-value--online">● 正常运行</span>
          </div>
        </div>
      </div>
    </div>
  );
}

function SmartParkTab() {
  const [selectedBuilding, setSelectedBuilding] = useState(null);
  const [showDetail, setShowDetail] = useState(false);
  const [viewMode, setViewMode] = useState('overview');
  const [weather, setWeather] = useState('sunny');
  const [timeOfDay, setTimeOfDay] = useState('noon');

  const handleBuildingClick = useCallback((building) => {
    setSelectedBuilding(building);
    setShowDetail(true);
  }, []);

  const handleBuildingSelect = useCallback((building) => {
    setSelectedBuilding(building);
    setShowDetail(true);
  }, []);

  const handleCloseDetail = useCallback(() => {
    setShowDetail(false);
    setSelectedBuilding(null);
  }, []);

  const handleViewChange = useCallback((view) => {
    setViewMode(view);
  }, []);

  const handleWeatherChange = useCallback((w) => {
    setWeather(w);
  }, []);

  const handleTimeChange = useCallback((t) => {
    setTimeOfDay(t);
  }, []);

  return (
    <div className="smart-park__tab">
      {/* 3D 画布区域 */}
      <div className="smart-park__canvas-area">
        <Canvas
          camera={{ position: [10, 8, 10], fov: 50 }}
          shadows
          gl={{ antialias: true, alpha: false }}
        >
          <Scene
            onBuildingClick={handleBuildingClick}
            selectedBuilding={selectedBuilding}
            viewMode={viewMode}
            weather={weather}
            timeOfDay={timeOfDay}
          />
        </Canvas>
      </div>

      {/* 右侧数据面板 */}
      <DataPanel
        selectedBuilding={selectedBuilding}
        parkData={PARK_DATA}
        onBuildingSelect={handleBuildingSelect}
      />

      {/* 左上角环境监测 */}
      <div className="env-monitor">
        <div className="env-monitor__item">
          <span className="env-monitor__icon">🌡️</span>
          <span className="env-monitor__value">24°C</span>
        </div>
        <div className="env-monitor__divider" />
        <div className="env-monitor__item">
          <span className="env-monitor__icon">💧</span>
          <span className="env-monitor__value">65%</span>
        </div>
        <div className="env-monitor__divider" />
        <div className="env-monitor__item">
          <span className="env-monitor__icon">💨</span>
          <span className="env-monitor__value">3级</span>
        </div>
        <div className="env-monitor__divider" />
        <div className="env-monitor__item">
          <span className="env-monitor__icon">🌫️</span>
          <span className="env-monitor__value">良</span>
        </div>
      </div>

      {/* 左侧建筑详情弹窗 */}
      {showDetail && selectedBuilding && (
        <BuildingDetailPopup
          building={selectedBuilding}
          onClose={handleCloseDetail}
        />
      )}

      {/* 底部工具栏 */}
      <Toolbar
        onViewChange={handleViewChange}
        currentView={viewMode}
        weather={weather}
        onWeatherChange={handleWeatherChange}
        timeOfDay={timeOfDay}
        onTimeChange={handleTimeChange}
      />

      {/* 标题 */}
      <div className="smart-park__title">
        <div className="smart-park__title-main">数字孪生智慧园区</div>
        <div className="smart-park__title-sub">Digital Twin Smart Park</div>
      </div>
    </div>
  );
}

export default SmartParkTab;
