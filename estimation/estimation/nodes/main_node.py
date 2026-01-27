from __future__ import annotations
import rclpy
from estimation.utils.estimation_core import EstimationMainNode

def main(args=None):
    rclpy.init(args=args); node=EstimationMainNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node._executor.shutdown(wait=False); node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()
