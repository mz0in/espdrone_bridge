#include "ac_lee_position_controller_esp.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filed_lee");
    field_esp_base* base = new field_esp_base(argc, argv);
    ros::NodeHandle n("~");
    base->counter_th = 100;
    base->use_position =  true;
    n.param("ppx", base->ppx, 6.);
    n.param("ppy", base->ppy, 6.);
    n.param("ppz", base->ppz, 12.);
    n.param("pvx", base->pvx, 4.7);
    n.param("pvy", base->pvy, 4.7);
    n.param("pvz", base->pvz, 8.0);
    n.param("mass", base->mass, 2.31);
    n.param("pth", base->pth, 0.8);
    n.param("angx", base->angx, -3.);
    n.param("angy", base->angy, -3.);
    n.param("angz", base->angz, -3.);
    std::cout << BOLDGREEN << "l 10 of field_esp_base_node.cpp " << base->use_position << std::endl << RESET;
    base->start_process();
    delete base;
    return 0;
}

