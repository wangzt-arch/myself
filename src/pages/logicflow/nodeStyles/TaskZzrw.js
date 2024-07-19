// 矩形
import { RectNode, RectNodeModel } from '@logicflow/core'

class TaskNodeModel extends RectNodeModel {
  initNodeData(data) {
    super.initNodeData(data)
    // this.width = 96
    this.height = 36
    this.radius = 4
  }

  getNodeStyle() {
    const style = super.getNodeStyle()
    style.fill = 'rgba(64, 226, 255,0.5)'
    style.stroke = 'rgba(64, 226, 255,1)'
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
}

const taskZzrw = {
  type: 'TaskZzrw',
  view: RectNode,
  model: TaskNodeModel
}
export default taskZzrw
