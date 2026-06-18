// 矩形
import { RectNode, RectNodeModel } from '@logicflow/core'

class TaskNodeModel extends RectNodeModel {
  initNodeData(data) {
    super.initNodeData(data)
    this.height = 36
    this.radius = 4
  }

  getNodeStyle() {
    const style = super.getNodeStyle()
    style.fill = 'rgba(244, 63, 94,0.5)'
    style.stroke = 'rgba(244, 63, 94,1)'
    style.strokeWidth = 1
    return style
  }

  getTextStyle() {
    const style = super.getTextStyle()
    style.fontSize = 14
    style.color = '#000'
    return style
  }
  getDefaultAnchor() {
    const { width, height, x, y, id } = this;
    return [
      {
        x: x,
        y: y - height / 2,
        type: 'top',
        id: `${id}_0`,
      },
      {
        x: x,
        y: y + height / 2,
        type: 'bottom',
        id: `${id}_1`,
      },
      {
        x: x - width / 2,
        y: y,
        type: 'left',
        id: `${id}_2`,
      },
      {
        x: x + width / 2,
        y: y,
        type: 'right',
        id: `${id}_3`,
      },
    ];
  }
}

const taskZzwj = {
  type: 'TaskZzwj',
  view: RectNode,
  model: TaskNodeModel
}
export default taskZzwj
