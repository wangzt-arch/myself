import './index.css'
import { useEffect, useRef, useState } from "react";
import { Menu, DndPanel, SelectionSelect,Snapshot } from "@logicflow/extension";
import LogicFlow from "@logicflow/core";
import "@logicflow/core/dist/style/index.css";
import "@logicflow/extension/lib/style/index.css";
import { toolMap } from './toolsData';
import PositionsTools from './components/positionTools';
import SaveTools from './components/saveTools'
import Header from "../../components/Header";

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


export default function LogicFlowCanvas() {

  useEffect(() => {
    initCanvas()
  }, [])
  const [lf, setLf] = useState(null)
  const lfRef = useRef(null)

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
          text: "删除",
          callback: (node) => {
            lf.deleteNode(node.id);
          },
        },
      ],
      // 删除默认的边右键菜单
      edgeMenu: [
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
    ]);
    lf.on('node:dnd-add', ({ data }) => {
    })

    lf.on("node:click", ({ data }) => {
    });

    lf.on('selection:selected', () => {
      lf.extension.selectionSelect.closeSelectionSelect()
    })

    //渲染
    lf.render()
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
      default:
        break;
    }
  }
  const renderRightTool = () => {
    return toolMap.map(item =>
      <div key={item.name} onClick={() => rightToolsClick(item.name)} className='tools-item'>
        <img width='30px' height='30px' src={require('./img/' + item.icon)} alt="" title={item.desc} />
      </div>
    )
  }
  return (
    <div className="logicflow-box">
      <Header></Header>
      <div className="logicflow-box__canvas" ref={lfRef}></div>
      <div className="right-tools">
        {renderRightTool()}
      </div>
      <PositionsTools lf={lf} />
      <SaveTools lf={lf} />
    </div>
  )
}
