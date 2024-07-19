// 矩形
import { RectNode, RectNodeModel } from '@logicflow/core'

class TaskNodeModel extends RectNodeModel {
  initNodeData(data) {
    super.initNodeData(data)
    this.width = 120
    this.height = 50
    this.radius = 4
  }
  getDefaultAnchor() {
    const { height, x, y, id } = this;
    return [
      {
        x: x,
        y:y-height/2,
        type: 'top',
        id: `${id}_0`,
      },
      {
        x:x,
        y:y+height/2,
        type: 'bottom',
        id: `${id}_1`,
      },
    ];
  }

  getNodeStyle() {
    const style = super.getNodeStyle()
    style.fill = '#0C97D850'
    style.stroke = '#0C97D8'
    style.strokeWidth = 1
    return style
  }

  getTextStyle() {
    const style = super.getTextStyle()
    style.fontSize = 14
    style.color = '#000'
    return style
  }
}

const taskNode =  {
  type: 'TaskNode',
  view: RectNode,
  model: TaskNodeModel
}
export default taskNode
