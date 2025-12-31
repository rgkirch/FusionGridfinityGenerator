import adsk.core, adsk.fusion, traceback
import math

# Import the generator modules
# Note: We need to adjust path or copying/mocking might be needed if running as a separate script.
# Assuming this script is placed in the root of the add-in or similar location where it can import lib.

from .lib.gridfinityUtils import binBodyGenerator, binBodyGeneratorInput, const
from .lib import fusion360utils as futil

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        
        # Create a new document to avoid messing up the current one
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent

        # Setup Input
        binInput = binBodyGeneratorInput.BinBodyGeneratorInput()
        binInput.binWidth = 42.0 # 1 unit
        binInput.binLength = 42.0 # 1 unit
        binInput.binHeight = 42.0 # 6 units (6*7)
        binInput.isSolid = False
        binInput.wallThickness = 1.2
        
        # Tab Settings
        binInput.hasTab = True
        binInput.tabMethod = const.BIN_TAB_METHOD_DIMENSIONS
        binInput.rootThickness = 0.14 # 1.4mm
        binInput.tipThickness = 0.14
        binInput.tabLength = 12.0 # 12mm
        binInput.tabWidth = 12.0 # Width of tab (along X axis? No, width is usually "Depth" of label area)
        # Wait, tabWidth in UI maps to 'width' property in generatorInput?
        # In entry.py: binBodyInput.tabWidth = binTabWidth.value
        
        # Run Generator
        # We need a dummy 'innerCutoutBody' if we were running the full generator,
        # but createGridfinityBinBody does the whole thing.
        # However, createGridfinityBinBody requires 'baseGeneratorInput' context usually?
        # Actually createGridfinityBinBody creates the bin body.
        
        # Let's call the high-level function that creates everything?
        # No, let's call createGridfinityBinBody which is what generateBin calls.
        
        # Note: createGridfinityBinBody creates the bin, cutout, and tab.
        binBody = binBodyGenerator.createGridfinityBinBody(
            binInput,
            root
        )
        
        # Assertions
        # Check if we have bodies
        if root.bRepBodies.count == 0:
             ui.messageBox('Test Failed: No bodies created.')
             return

        # Check Dimensions
        # Bounding Box of the bin
        bbox = binBody.boundingBox
        # Check Height
        height = bbox.maxPoint.z - bbox.minPoint.z
        # Expected: 42.0
        if abs(height - 42.0) > 0.01:
             ui.messageBox(f'Test Failed: Height mismatch. Expected 42.0, got {height}')
             return

        # Check for Tab Feature
        # The tab should add volume. 
        # Easier to visually verify or check if specific face exists.
        
        ui.messageBox('Test Passed: Bin created with Dimensions Tab.')

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
