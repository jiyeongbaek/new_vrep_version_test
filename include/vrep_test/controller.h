#pragma once
#include <iostream>
#include <vrep_test/model/rbdl.h>

class FrankaController
{
    FrankaController(std::shared_ptr<FrankaModelUpdater> &model);
};
