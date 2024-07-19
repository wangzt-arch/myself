import './index.css'


const text = ["垂直居中", "水平居中", "垂直分布", "水平分布"]


export default function positionsTools(props) {
  const { lf } = props


  const layout = (index) => {
    // 需要组织为以下的数据结构后调用autoLayout方法
    var selNodes = [];

    lf.graphModel.selectElements.forEach((value, key) => {
      if (value.BaseType === 'node') {
        selNodes.push({
          node: value,
          position: { x: value.x, y: value.y },
        });
      }
    });

    switch (index) {
      case 0:
        autoLayout(selNodes, "vc");
        break;
      case 1:
        autoLayout(selNodes, "hc");
        break;
      case 2:
        autoLayout(selNodes, "v");
        break;
      case 3:
        autoLayout(selNodes, "h");
        break;
      default:
        break;
    }
  }

  const autoLayout = (selectedNodes, mode) => {
    /* 'v': 垂直分布，'h': 水平分布，'vc': 垂直居中, 'hc': 水平居中 */
    var gm = lf.graphModel;
    let nodeList = []

    function changeNodePosition(node, x, y) {
      if (node) {
        var moveOffset = { x: 0, y: 0 };

        if (x) {
          moveOffset.x = x - node.x;
        }

        if (y) {
          moveOffset.y = y - node.y;
        }

        gm.moveNode(node.id, moveOffset.x, moveOffset.y);
        nodeList.push(lf.getNodeModelById(node.id))
      }
    }

    if (selectedNodes.length <= 1) {
      return;
    }

    var minRect = { x: Number.MAX_VALUE, y: Number.MAX_VALUE };
    var maxRect = { x: Number.MIN_VALUE, y: Number.MIN_VALUE };

    for (var i = 0; i < selectedNodes.length; ++i) {
      if (minRect.x > selectedNodes[i].position.x) {
        minRect.x = selectedNodes[i].position.x;
      }
      if (minRect.y > selectedNodes[i].position.y) {
        minRect.y = selectedNodes[i].position.y;
      }

      if (maxRect.x < selectedNodes[i].position.x) {
        maxRect.x = selectedNodes[i].position.x;
      }
      if (maxRect.y < selectedNodes[i].position.y) {
        maxRect.y = selectedNodes[i].position.y;
      }
    }

    switch (mode) {
      case "v": // 垂直分布
        {
          // 按y值进行排序
          selectedNodes.sort(function (a, b) {
            return a.position.y - b.position.y;
          });

          let yStep = (maxRect.y - minRect.y) / (selectedNodes.length - 1);

          // 从小到大调整每个结点的位置
          for (let i = 0; i < selectedNodes.length; ++i) {
            changeNodePosition(
              selectedNodes[i].node,
              // null,
              selectedNodes[i].node.x,
              minRect.y + yStep * i
            );
          }
        }
        break;

      case "h": // 水平分布
        {
          // 按x值进行排序
          selectedNodes.sort(function (a, b) {
            return a.position.x - b.position.x;
          });
          let xStep = (maxRect.x - minRect.x) / (selectedNodes.length - 1);
          // 从小到大调整每个结点的位置
          for (let i = 0; i < selectedNodes.length; ++i) {
            changeNodePosition(
              selectedNodes[i].node,
              minRect.x + xStep * i,
              // null
              selectedNodes[i].node.y,
            );
          }
        }
        break;

      case "vc": // 垂直居中
        {
          // 按y值进行排序
          selectedNodes.sort(function (a, b) {
            return a.position.y - b.position.y;
          });

          // let yStep = (maxRect.y - minRect.y) / (selectedNodes.length - 1);

          let xCenter = (maxRect.x + minRect.x) * 0.5;

          // 从小到大调整每个结点的位置
          for (let i = 0; i < selectedNodes.length; ++i) {
            changeNodePosition(
              selectedNodes[i].node,
              xCenter,
              // minRect.y + yStep * i
              selectedNodes[i].node.y,
            );
          }
        }
        break;

      case "hc": // 水平居中
        {
          // 按x值进行排序
          selectedNodes.sort(function (a, b) {
            return a.position.x - b.position.x;
          });

          // let xStep = (maxRect.x - minRect.x) / (selectedNodes.length - 1);
          let yCenter = (minRect.y + maxRect.y) * 0.5;

          // 从小到大调整每个结点的位置
          for (let i = 0; i < selectedNodes.length; ++i) {
            changeNodePosition(
              selectedNodes[i].node,
              // minRect.x + xStep * i,
              selectedNodes[i].node.x,
              yCenter
            );
          }
        }
        break;
      default:
        break;  
    }
  }
  if (!lf) return <div></div>


  return (
    <div className="positions-tools">
      {
        text.map((item, index) => {
          return <div className='aux-box' key={index}>
            <div className='item' onClick={() => layout(index)}>
              <div className="img"></div>
              <div className="text">{item}</div>
            </div>
          </div>
        })
      }

    </div>
  )
}
