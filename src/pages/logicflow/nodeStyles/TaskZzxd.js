// 矩形
import { RectNode, RectNodeModel } from '@logicflow/core'

const DEFAULT_COLOR = {
  fill: 'rgba(23, 187, 128,0.5)',
  stroke: 'rgba(23, 187, 128,1)'
}

class TaskNodeModel extends RectNodeModel {
  initNodeData(data) {
    super.initNodeData(data)
    this.height = 36
    this.radius = 4
    this.customColor = data.color || DEFAULT_COLOR
  }

  getNodeStyle() {
    const style = super.getNodeStyle()
    const color = this.properties.customColor || this.customColor
    style.fill = color.fill
    style.stroke = color.stroke
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

const taskZzxd = {
  type: 'TaskZzxd',
  view: RectNode,
  model: TaskNodeModel
}
export default taskZzxd
