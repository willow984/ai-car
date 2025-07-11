import signal
from main import app, initialize_app, cleanup
from config import FLASK_PORT

if __name__ == '__main__':
    # 绑定信号处理函数，确保Ctrl+C可以优雅地关闭程序
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    
    # 初始化应用所需的所有服务
    initialize_app()
    
    print("\n--- AI小车控制服务已启动 ---")
    print(f"* 访问地址 1: http://127.0.0.1:{FLASK_PORT}")
    print(f"* 请将浏览器指向你小车的实际IP地址, 例如: http://192.168.10.103:{FLASK_PORT}")
    print("-----------------------------\n")

    # 运行Flask应用
    # debug=False 和 use_reloader=False 在生产或稳定部署时是推荐的设置
    app.run(host='0.0.0.0', port=FLASK_PORT, debug=False, threaded=True, use_reloader=False)
