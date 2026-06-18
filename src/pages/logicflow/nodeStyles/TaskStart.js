// 圆形节点 - 用于开始/结束
import { CircleNode, CircleNodeModel } from '@logicflow/core'

const DEFAULT_COLOR = {
  fill: 'rgba(0, 200, 255,0.5)',
  stroke: 'rgba(0, 200, 255,1)'
}

class CircleTaskModel extends CircleNodeModel {
  initNodeData(data) {
    super.initNodeData(data)
    this.r = 28
    this.customColor = data.color || DEFAULT_COLOR
  }
  getNodeStyle() {
    const style = super.getNodeStyle()
    const color = this.properties.customColor || this.customColor
    style.fill = color.fill
    style.stroke = color.stroke
    style.strokeWidth = 1.5
    return style
  }
  getTextStyle() {
    const style = super.getTextStyle()
    style.fontSize = 13
    style.color = '#000'
    return style
  }
  getDefaultAnchor() {
    const { x, y, r, id } = this
    return [
      { x: x, y: y - r, type: 'top', id: `${id}_0` },
      { x: x + r, y: y, type: 'right', id: `${id}_1` },
      { x: x, y: y + r, type: 'bottom', id: `${id}_2` },
      { x: x - r, y: y, type: 'left', id: `${id}_3` },
    ]
  }
}

const taskStartNode = {
  type: 'TaskStart',
  view: CircleNode,
  model: CircleTaskModel
}
export default taskStartNode
