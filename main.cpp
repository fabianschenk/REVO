/**
* This file is part of REVO.
*
* Copyright (C) 2014-2017 Schenk Fabian <schenk at icg dot tugraz dot at> (Graz University of Technology)
* For more information see <https://github.com/fabianschenk/REVO/>
*
* REVO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* REVO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with REVO. If not, see <http://www.gnu.org/licenses/>.
*/
#include "system/system.h"
#include <memory>
int main(int argc, char ** argv)
{
    if (argc < 2) //first is name of program
    {
        I3D_LOG(i3d::error) << "Not enough input arguments: REVO configFile.yaml datasetFile.yaml";
        exit(0);
    }

    //try to convert argument to string
    const std::string settingsFile = argv[1], datasetFile = argv[2];
    int nRuns = 0;
    //now start the system
    while (true)
    {
        REVO revoSystem(settingsFile,datasetFile,nRuns);

        if (!revoSystem.start())
        {
            I3D_LOG(i3d::info) << "Finished all datasets!";
            return EXIT_SUCCESS;
        }
        nRuns++;
        I3D_LOG(i3d::info) << "nRuns" << nRuns;

    }
    return EXIT_SUCCESS;
}
