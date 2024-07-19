// 折线
import { PolylineEdge, PolylineEdgeModel } from '@logicflow/core'

class TaskLineModel extends PolylineEdgeModel {
  getEdgeStyle() {
    const style = super.getEdgeStyle()
    style.stroke = '#00C0FF'
    return style
  }
}
const taskLine = {
  type: 'TaskLine',
  view: PolylineEdge,
  model: TaskLineModel
}

export default taskLine
 