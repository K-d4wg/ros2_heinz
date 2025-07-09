import rclpy

def main(args=None):
    rclpy.init(args=args)
    print("example")

    #controller = LLController()
    #rclpy.spin(controller)
    #controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()