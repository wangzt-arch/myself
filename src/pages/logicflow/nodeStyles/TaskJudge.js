// 菱形节点 - 用于判断
import { DiamondNode, DiamondNodeModel } from '@logicflow/core'

const DEFAULT_COLOR = {
  fill: 'rgba(245, 158, 11,0.5)',
  stroke: 'rgba(245, 158, 11,1)'
}

class DiamondTaskModel extends DiamondNodeModel {
  initNodeData(data) {
    super.initNodeData(data)
    this.rx = 60
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

const taskJudgeNode = {
  type: 'TaskJudge',
  view: DiamondNode,
  model: DiamondTaskModel
}
export default taskJudgeNode
