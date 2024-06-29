/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       CWood-sdf                                                 */
/*    Created:      2/15/2023, 5:40:09 PM                                     */
/*    Description:  idk                                                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "robot-config.h"

using namespace vex;
controller Controller = controller(primary);
competition Competition;
void autonInit()
{
    cout << "Auton Init" << endl;
    cout << "Auton Init Done" << endl;
}
void autonomous()
{
    while (!Auton::isSelected())
    {
        s(100);
    }

    // Auton timer system, used in testing
    auto startTime = Brain.Timer.system();
    autonInit();
    // Put auton call here
    Auton::callAuton();
    // wc.driveTo(1, 1);
    // Print time
    cout << "Auton Time: " << (Brain.Timer.system() - startTime) / 1000.0 << endl;
}
void drivercontrol()
{
    // Protection from multiple instances of drivercontrol running
    // Is true if there is no instance of drivercontrol running
    static bool allEmpty = false;
    // An array of the current and past instances of drivercontrol
    // The pair::first is whether the instance has exited or not
    // The pair::second is a pointer to the primary bool a few lines down
    static vector<pair<bool, bool *>> countsExist = {};
    // The count of drivercontrol instances
    static int count = 0;
    // Is true if this drivercontrol instance should be running
    bool primary = count == 0 || allEmpty ? true : false;
    // Push back the array
    countsExist.push_back({true, &primary});
    // The index of this drivercontrol instance in countsExist
    int localCount = count;
    count++;
    DriveController dc = DriveController(&chassis, 20);
    while (1)
    {
        // Place driver code in here
        if (primary)
        {
            dc.driveTank(Controller.Axis3, Controller.Axis2);
            // Driver code here
        }
        else
        {
        }
        s(10);
    }
    // Let other instances know that this drivercontrol can't run
    countsExist[localCount].first = false;
    // Search for a working drivercontrol instance, and set it to working
    // If it's not working, allEmpty will be true
    for (auto [exist, ptr] : countsExist)
    {
        if (exist)
        {
            *ptr = true;
            allEmpty = false;
            break;
        }
        else
        {
            allEmpty = true;
        }
    }
}
bool init = false;
void updatePos()
{
    while (true)
    {
        positioner.update();
        s(10);
    }
}
int main()
{

    // Init has to be in thread, otherwise it won't work with comp switch
    thread initThread = thread([]()
                               {
        v5_lv_init();
        cout << "<< Lvgl initialized >>" << endl;
        positioner.init();
        positioner.setPos({0, 0}, 0);
        cout << "<< Odometry initialized >>" << endl;
        testDeviceConnection();
        // testDriveConfiguration();
        cout << "<< Motor connection test complete >>" << endl;
        wc.path
            .setK(1.4)
            .setMaxAcc(200)
            .setMaxDAcc(120);
        cout << "<< Chassis initialized >>" << endl;

        BosFn::addNewFn(testConnection);
        BosFn::addNewFn(VariableConfig::drawAll);
        BosFn::addNewFn(windowsLoader);
        BosFn::useTransparentScreenSwitchButtons();
        cout << "<< BrainOS functions initialized >>" << endl;
        init = true; });
    while (!init)
    {
        s(100);
    }
    VariableConfig setAlliance = VariableConfig({"red", "blue"}, "Alliance", 0, [](int i)
                                                {
        if (i == 0) {
        wc.setRed();
        }
        else {
        wc.setBlue();
        } });

    thread posUpdate = thread(updatePos);
    // Awesome brain screen control thread
    thread loader = thread([]()
                           { BosFn::runBrainOS(); });
                           
    Competition.autonomous(autonomous);
    Competition.drivercontrol(drivercontrol);

    // Prevent main from exiting
    while (1)
    {
        s(300);
    }
}
