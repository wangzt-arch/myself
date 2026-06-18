// 椭圆节点 - 用于状态/结果
import { EllipseNode, EllipseNodeModel } from '@logicflow/core'

const DEFAULT_COLOR = {
  fill: 'rgba(74, 222, 128,0.5)',
  stroke: 'rgba(74, 222, 128,1)'
}

class EllipseTaskModel extends EllipseNodeModel {
  initNodeData(data) {
    super.initNodeData(data)
    this.rx = 70
    this.ry = 30
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
    const { x, y, rx, ry, id } = this
    return [
      { x: x, y: y - ry, type: 'top', id: `${id}_0` },
      { x: x + rx, y: y, type: 'right', id: `${id}_1` },
      { x: x, y: y + ry, type: 'bottom', id: `${id}_2` },
      { x: x - rx, y: y, type: 'left', id: `${id}_3` },
    ]
  }
}

const taskStateNode = {
  type: 'TaskState',
  view: EllipseNode,
  model: EllipseTaskModel
}
export default taskStateNode
