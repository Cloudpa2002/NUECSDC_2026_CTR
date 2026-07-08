#include <iostream>

void printHelp() {
    std::cout << "\n";
    std::cout << "1. ros2 node list : 查看节点列表*\n";
    std::cout << "2. ros2 node info /node_name :  查看节点信息\n";
    std::cout << "3. ros2 service list ： 查看服务列表*\n";
    std::cout << "4. ros2 service type /xxx : 查看服务接口类型*\n";
    std::cout << "5. ros2 interface show xxx/yyy ： 查看接口类型的参数*\n";
    std::cout << "\n";
    std::cout << "6. ros2 service call /参数1 /参数2 \"{参数3:x , 参数4:y}\" : 手动调用函数\n";
    std::cout << "\t 参数1：服务点 ； 参数2：接口参数类型 ； 参数3/4：接口参数类型中定义的参数\n";
    std::cout << "\n";
    std::cout << "7. ros2 topic echo <topic_name> : 在控制台显示主题消息\n";
    std::cout << "\n";
    std::cout << "8. ros2 topic pub <topic_name> <message_type> <message_content> : 通过终端发布topic\n";
    std::cout << "\t ros2 topic pub -1 <topic_name> <message_type> <message_content>\n";
    std::cout << "\t ros2 topic pub -t 5 <topic_name> <message_type> <message_content>\n";
    std::cout << "\t ros2 topic pub -r 5 <topic_name> <message_type> <message_content>\n";
    std::cout << "默认循环发布，频率为1Hz，参数-1只发布一次，参数-t 5循环发布5次结束，参数-r 5以5Hz的频率循环发布\n";
    std::cout << "\n";
    std::cout << "9. ros2 service find xxx/yyy : 查看接口的服务类型\n";
    std::cout << "\n";
}

int main() {
    printHelp();
    return 0;
}