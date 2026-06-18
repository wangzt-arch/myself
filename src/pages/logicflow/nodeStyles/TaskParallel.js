// 多边形节点 - 用于并行/汇聚
import { PolygonNode, PolygonNodeModel } from '@logicflow/core'

const DEFAULT_COLOR = {
  fill: 'rgba(139, 92, 255,0.5)',
  stroke: 'rgba(139, 92, 255,1)'
}

class PolygonTaskModel extends PolygonNodeModel {
  initNodeData(data) {
    super.initNodeData(data)
    this.points = [
      [50, 0],
      [100, 35],
      [80, 80],
      [20, 80],
      [0, 35]
    ]
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
    const { x, y, id } = this
    return [
      { x: x, y: y - 40, type: 'top', id: `${id}_0` },
      { x: x + 50, y: y, type: 'right', id: `${id}_1` },
      { x: x, y: y + 40, type: 'bottom', id: `${id}_2` },
      { x: x - 50, y: y, type: 'left', id: `${id}_3` },
    ]
  }
}

const taskParallelNode = {
  type: 'TaskParallel',
  view: PolygonNode,
  model: PolygonTaskModel
}
export default taskParallelNode
