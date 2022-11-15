#include "GUIUpdater.h"

GUIUpdater* GUIUpdater::_updaterInstance = NULL;

void GUIUpdater::init()
{
    GUIUpdater* guiUpdaterPtr = new GUIUpdater();
    GUIUpdater::_updaterInstance = guiUpdaterPtr;
}
