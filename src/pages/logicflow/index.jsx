import './index.css'
import { useEffect, useRef, useState } from "react";
import { Menu, DndPanel, SelectionSelect,Snapshot } from "@logicflow/extension";
import LogicFlow from "@logicflow/core";
import "@logicflow/core/dist/style/index.css";
import "@logicflow/extension/lib/style/index.css";
import { toolMap } from './toolsData';
import PositionsTools from './components/positionTools';
import SaveTools from './components/saveTools'
import { defaultDevelopmentFlow } from './defaultFlowData';
import { TrashIcon } from './img/TrashIcon';
import ColorPicker from './components/colorPicker';

const modulesFiles = require.context("./nodeStyles", true, /\.js$/);

const modules = [];
modulesFiles.keys().forEach((key) => {
  const module = modulesFiles(key).default;
  modules.push(module);
});

LogicFlow.use(Menu);
LogicFlow.use(DndPanel);
LogicFlow.use(SelectionSelect);
LogicFlow.use(Snapshot);

const rgbToHex = (rgb) => {
  if (rgb.startsWith('#')) return rgb;
  const match = rgb.match(/rgba?\((\d+),\s*(\d+),\s*(\d+)/);
  if (!match) return '#00c8ff';
  const r = parseInt(match[1]).toString(16).padStart(2, '0');
  const g = parseInt(match[2]).toString(16).padStart(2, '0');
  const b = parseInt(match[3]).toString(16).padStart(2, '0');
  return `#${r}${g}${b}`;
};

const hexToRgb = (hex) => {
  const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  return result ? {
    r: parseInt(result[1], 16),
    g: parseInt(result[2], 16),
    b: parseInt(result[3], 16)
  } : null;
};

export default function LogicFlowCanvas() {

  useEffect(() => {
    initCanvas()
  }, [])
  const [lf, setLf] = useState(null)
  const lfRef = useRef(null)
  const [colorPicker, setColorPicker] = useState({ show: false, nodeId: null, initialColor: '#00c8ff' })

  const initCanvas = () => {
    const lf = new LogicFlow({
      container: lfRef.current,
      // edgeType: "TaskLine",
      edgeTextEdit: false,
      adjustEdge: true,
      autoExpand: true,
      stopMoveGraph: false,
      edgeType: 'bezier',
      grid: {
        size: 10,
        type: 'dot'
      },
      stopScrollGraph: true,
      multipleSelectKey: "shift",
      keyboard: {
        enabled: true,
        shortcuts: [
      {
        keys: ["delete"],
        callback: () => {
            const elements = lf.getSelectElements(true);
            lf.clearSelectElements();
            elements.edges.forEach((edge) => lf.deleteEdge(edge.id));
            elements.nodes.forEach((node) => lf.deleteNode(node.id));
        },
      },
    ],

      }
    });
    setLf(lf)
    modules.forEach(item => {
      lf.register(item);
    })
    lf.setTheme({
      outline: {
        fill: "transparent",
        stroke: "#0096ff",
        strokeDasharray: "3,3",
        hover: {
          stroke: "#0096ff",
        },
      },
    });
    //选中区域菜单
    lf.extension.menu.setMenuByType({
      type: "lf:defaultSelectionMenu",
      menu: [],
    });

    lf.extension.menu.setMenuConfig({
      // 覆盖默认的节点右键菜单
      nodeMenu: [
        {
          text: "添加/修改文本",
          callback: (node) => {
            lf.editText(node.id);
          },
        },
        {
          text: "修改节点颜色",
          callback: (node) => {
            const nodeModel = lf.getNodeModelById(node.id);
            const currentColor = nodeModel?.customColor?.stroke || '#00c8ff';
            setColorPicker({
              show: true,
              nodeId: node.id,
              initialColor: rgbToHex(currentColor)
            });
          },
        },
        {
          text: "删除",
          callback: (node) => {
            lf.deleteNode(node.id);
          },
        },
      ],
      // 删除默认的边右键菜单
      edgeMenu: [
        {
          text: "添加/修改文本",
          callback: (edge) => {
            lf.editText(edge.id);
          },
        },
        {
          text: "删除",
          callback: (edge) => {
            lf.deleteEdge(edge.id);
          },
        },
      ],
    });

    lf.extension.dndPanel.setPatternItems([
      {
        type: "TaskStart",
        text: "起",
        className: "task-node-start",
        label: "起"
      },
      {
        type: "TaskZzrw",
        text: "1",
        className: "task-node1",
        label: "1"
      },
      {
        type: "TaskZzxd",
        text: "2",
        className: "task-node2",
        label: "2"
      },
      {
        type: "TaskZzhd",
        text: "3",
        className: "task-node3",
        label: "3"
      },
      {
        type: "TaskJudge",
        text: "判",
        className: "task-node-judge",
        label: "判"
      },
      {
        type: "TaskState",
        text: "态",
        className: "task-node-state",
        label: "态"
      },
      {
        type: "TaskParallel",
        text: "并",
        className: "task-node-parallel",
        label: "并"
      },
    ]);
    lf.on('node:dnd-add', ({ data }) => {
    })

    lf.on("node:click", ({ data }) => {
    });

    lf.on('selection:selected', () => {
      lf.extension.selectionSelect.closeSelectionSelect()
    })

    // 渲染默认开发流程
    lf.render(defaultDevelopmentFlow)
    requestAnimationFrame(() => {
      lf.fitView(80, 80);
    });
  }
  const rightToolsClick = (type) => {
    switch (type) {
      case 'redo':
        lf.redo()
        break;
      case 'undo':
        lf.undo()
        break;
      case 'zoomIn':
        lf.zoom(true);
        break;
      case 'zoomOut':
        lf.zoom(false);
        break;
      case 'fitView':
        lf.fitView(100, 100);
        break;
      case 'selection':
        lf.extension.selectionSelect.openSelectionSelect()
        break;
      case 'clear':
        if (window.confirm('确定要清空画布吗？此操作不可撤销。')) {
          lf.clearData();
        }
        break;
      default:
        break;
    }
  }
  const renderRightTool = () => {
    return toolMap.map(item => {
      const isSvg = item.name === 'clear';
      return (
        <div
          key={item.name}
          onClick={() => rightToolsClick(item.name)}
          className={`tools-item ${isSvg ? 'tools-item--clear' : ''}`}
          title={item.desc}
        >
          {isSvg ? (
            <TrashIcon size={30} />
          ) : (
            <img width='30px' height='30px' src={require('./img/' + item.icon)} alt="" title={item.desc} />
          )}
        </div>
      );
    });
  }
  return (
    <div className="logicflow-box">
      <div className="logicflow-box__canvas" ref={lfRef}></div>
      <div className="right-tools">
        {renderRightTool()}
      </div>
      <PositionsTools lf={lf} />
      <SaveTools lf={lf} />
      {colorPicker.show && (
        <ColorPicker
          initialColor={colorPicker.initialColor}
          onConfirm={(hex) => {
            if (colorPicker.nodeId && lf) {
              const rgb = hexToRgb(hex);
              if (rgb) {
                lf.setProperties(colorPicker.nodeId, {
                  customColor: {
                    fill: `rgba(${rgb.r}, ${rgb.g}, ${rgb.b}, 0.9)`,
                    stroke: `rgb(${rgb.r}, ${rgb.g}, ${rgb.b})`
                  }
                });
              }
            }
            setColorPicker({ show: false, nodeId: null, initialColor: '#00c8ff' });
          }}
          onCancel={() => {
            setColorPicker({ show: false, nodeId: null, initialColor: '#00c8ff' });
          }}
        />
      )}
    </div>
  )
}
