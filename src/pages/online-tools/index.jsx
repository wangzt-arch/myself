import { useState } from 'react';
import { tools } from './tools/toolsData';
import JsonTool from './tools/JsonTool';
import RegexTool from './tools/RegexTool';
import './layout.css';

/* --------- 工具映射表（后续添加工具只需在此注册） --------- */
const toolComponents = {
  json: { component: JsonTool, tags: ['JSON', 'Formatter', 'Dev Tool'] },
  regex: { component: RegexTool, tags: ['Regex', 'Match', 'Dev Tool'] },
};

/* --------- 主组件 --------- */
export default function OnlineTools() {
  const [activeTab, setActiveTab] = useState(tools[0]?.id || 'json');
  const currentTool = tools.find((t) => t.id === activeTab) || tools[0];
  const ActiveComponent = toolComponents[activeTab]?.component || JsonTool;

  return (
    <div className="online-tools">
      <aside className="ot-sidebar">
        <div className="ot-sidebar__header">
          <div className="ot-sidebar__title">
            <span className="ot-sidebar__title-icon">{'>'}</span>
            <span>在线工具</span>
          </div>
          <p className="ot-sidebar__subtitle">开发者的实用工具集合</p>
        </div>
        <span className="ot-sidebar__nav-label">工具列表</span>
        <nav className="ot-sidebar__nav">
          {tools.map((tool) => (
            <button
              key={tool.id}
              className={`ot-sidebar__item ${activeTab === tool.id ? 'active' : ''}`}
              onClick={() => setActiveTab(tool.id)}
            >
              <div className="ot-sidebar__item-row">
                <span className="ot-sidebar__icon">{tool.icon}</span>
                <span className="ot-sidebar__name">{tool.name}</span>
              </div>
              <span className="ot-sidebar__desc">{tool.description}</span>
            </button>
          ))}
        </nav>
      </aside>

      <main className="ot-main">
        <header className="ot-tool-header">
          <div className="ot-tool-header__info">
            <h2 className="ot-tool-header__name">{currentTool?.name}</h2>
            <p className="ot-tool-header__desc">{currentTool?.description}</p>
            <div className="ot-tool-header__tags">
              {(toolComponents[activeTab]?.tags || []).map((t) => (
                <span key={t} className="ot-tool-tag">{t}</span>
              ))}
            </div>
          </div>
        </header>
        <div className="ot-tool-content">
          <ActiveComponent />
        </div>
      </main>
    </div>
  );
}
