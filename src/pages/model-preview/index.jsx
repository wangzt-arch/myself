import React, { useState } from "react";
import Header from "../../components/Header";
import ModelPreviewTab from "./ModelPreviewTab";
import SmartParkTab from "./smart-park";
import "./index.css";

const TABS = [
  { id: 'park', label: '🌆 智慧园区' },
  { id: 'model', label: '🏭 模型预览' },
];

function Preview() {
  const [activeTab, setActiveTab] = useState('park');

  return (
    <div className="model-preview">
      <Header />

      {/* Tab 切换栏 */}
      <div className="model-preview__tabs">
        {TABS.map((tab) => (
          <button
            key={tab.id}
            className={`model-preview__tab-btn ${activeTab === tab.id ? 'model-preview__tab-btn--active' : ''}`}
            onClick={() => setActiveTab(tab.id)}
          >
            {tab.label}
          </button>
        ))}
      </div>

      {/* 内容区域 */}
      <div className="model-preview__content">
        <div style={{ display: activeTab === 'park' ? 'block' : 'none', width: '100%', height: '100%' }} ><SmartParkTab /></div>
        <div style={{ display: activeTab === 'model' ? 'block' : 'none', width: '100%', height: '100%' }} ><ModelPreviewTab /></div>
      </div>
    </div>
  );
}

export default Preview;
