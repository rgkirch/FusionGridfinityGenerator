import adsk.core, adsk.fusion, traceback
import os
import math



from .const import BIN_TAB_EDGE_FILLET_RADIUS
from ...lib.gridfinityUtils import geometryUtils
from ...lib import fusion360utils as futil
from ...lib.gridfinityUtils import filletUtils
from . import const, combineUtils, faceUtils, commonUtils, sketchUtils, extrudeUtils, baseGenerator, edgeUtils
from .baseGeneratorInput import BaseGeneratorInput
from .binBodyTabGeneratorInput import BinBodyTabGeneratorInput
from ... import config

app = adsk.core.Application.get()
ui = app.userInterface

def getInnerCutoutScoopFace(
    innerCutout: adsk.fusion.BRepBody
    ) -> tuple[adsk.fusion.BRepFace, adsk.fusion.BRepFace]:
    innerCutoutYNormalFaces = [face for face in innerCutout.faces if faceUtils.isYNormal(face)]
    scoopFace = min(innerCutoutYNormalFaces, key=lambda x: x.boundingBox.minPoint.y)
    oppositeFace = max(innerCutoutYNormalFaces, key=lambda x: x.boundingBox.minPoint.y)
    return (scoopFace, oppositeFace)

def createGridfinityBinBodyTab(
    input: BinBodyTabGeneratorInput,
    targetComponent: adsk.fusion.Component,
):

    tabProfilePlaneInput: adsk.fusion.ConstructionPlaneInput = targetComponent.constructionPlanes.createInput()
    tabProfilePlaneInput.setByOffset(
        targetComponent.yZConstructionPlane,
        adsk.core.ValueInput.createByReal(input.origin.x)
        )
    tabProfilePlane = targetComponent.constructionPlanes.add(tabProfilePlaneInput)
    tabSketch: adsk.fusion.Sketch = targetComponent.sketches.add(tabProfilePlane)
    tabSketch.name = "label tab sketch"

    tabSketchLine = tabSketch.sketchCurves.sketchLines
    constraints = tabSketch.geometricConstraints
    dimensions = tabSketch.sketchDimensions
    
    # Calculate tab top edge start point
    # tab is centered on the bin width
    # tab is positioned on the bin length (y)
    # tab starts at bin top heightput.origin.z - input.topClearance
    tabTopEdgeHeight = input.origin.z - input.topClearance
    if input.tabMethod == const.BIN_TAB_METHOD_DIMENSIONS:
        # Dimensions based generation
        rootThickness = input.rootThickness
        tipThickness = input.tipThickness
        
        # Points in sketch coordinates (Y seems to be width/depth, X is height/Z)
        # Based on previous code:
        # tabTopEdgeHeight is X in sketch (Z in model)
        # input.origin.y is Y in sketch? No, see addByTwoPoints below.
        
        # Previous code:
        # line1 (Vertical in sketch?): (origin.x, origin.y, tabTopEdgeHeight) -> (origin.x, origin.y, tabTopEdgeHeight - actualTabHeight)
        # wait, modelToSketchSpace takes Point3D. 
        # The plane is offset from YZ plane by X.
        # So Sketch Plane X = Model Y, Sketch Plane Y = Model Z.
        
        # Let's look at the previous code's points:
        # P1: (origin.x, origin.y, tabTopEdgeHeight)
        # P2: (origin.x, origin.y, tabTopEdgeHeight - actualTabHeight)
        # P3: (origin.x, origin.y - actualTabWidth, tabTopEdgeHeight)
        # P4: (origin.x, origin.y - actualTabWidth, tabTopEdgeHeight) (Used in line 3)
        
        # Model coords:
        # origin.x is strictly X.
        # origin.y varies by -actualTabWidth.
        # tabTopEdgeHeight (Z) varies by -actualTabHeight.
        
        # So we want:
        # Top-Back (Wall): (origin.x, origin.y, tabTopEdgeHeight)
        # Top-Front (Tip): (origin.x, origin.y - input.width, tabTopEdgeHeight)
        # Bottom-Front (Tip): (origin.x, origin.y - input.width, tabTopEdgeHeight - tipThickness)
        # Bottom-Back (Wall): (origin.x, origin.y, tabTopEdgeHeight - rootThickness)
        
        # World Coordinates (X, Y, Z)
        y_back = input.origin.y
        y_front = input.origin.y - input.width
        z_top = tabTopEdgeHeight
        z_root_bottom = tabTopEdgeHeight - rootThickness
        z_tip_bottom = tabTopEdgeHeight - tipThickness
        
        # P_top_back: At wall, top edge.
        p_top_back = adsk.core.Point3D.create(input.origin.x, y_back, z_top)
        # P_top_front: At tip, top edge.
        p_top_front = adsk.core.Point3D.create(input.origin.x, y_front, z_top)
        # P_bot_front: At tip, bottom edge.
        p_bot_front = adsk.core.Point3D.create(input.origin.x, y_front, z_tip_bottom)
        # P_bot_back: At wall, bottom edge.
        p_bot_back = adsk.core.Point3D.create(input.origin.x, y_back, z_root_bottom)

        l_top = tabSketchLine.addByTwoPoints(tabSketch.modelToSketchSpace(p_top_back), tabSketch.modelToSketchSpace(p_top_front))
        l_front = tabSketchLine.addByTwoPoints(tabSketch.modelToSketchSpace(p_top_front), tabSketch.modelToSketchSpace(p_bot_front))
        l_bot = tabSketchLine.addByTwoPoints(tabSketch.modelToSketchSpace(p_bot_front), tabSketch.modelToSketchSpace(p_bot_back))
        # Debug logging
        futil.log(f"DEBUG: Tab Back Fillet Value: {input.tabFilletBack}")
        
        if input.tabFilletBack > 0.001:
            futil.log("DEBUG: Generating Back Fillet Geometry")
            # Back Fillet (Support) Logic
            # We want to fillet the corner between l_bot and the Wall (downwards).
            # Epsilon shift to ensure the wedge is 'inside' the cutout void for Intersection.
            epsilon = 0.0001
            y_back_shifted = y_back - epsilon
            
            # 1. Create temporary wall extension line downwards.
            p_wall_low = adsk.core.Point3D.create(input.origin.x, y_back_shifted, z_root_bottom - input.tabFilletBack * 10.0 - 1.0) # sufficiently low
            l_wall_temp = tabSketchLine.addByTwoPoints(tabSketch.modelToSketchSpace(p_bot_back), tabSketch.modelToSketchSpace(p_wall_low))
            
            # 2. Add Fillet between l_bot and l_wall_temp
            # Note: l_bot ends at p_bot_back. l_wall_temp starts at p_bot_back.
            # addFillet needs the lines to share the point or intersect.
            arc = tabSketch.sketchCurves.sketchArcs.addFillet(l_bot, l_bot.endSketchPoint.geometry, l_wall_temp, l_wall_temp.startSketchPoint.geometry, input.tabFilletBack)
            
            # 3. Clean up
            # l_bot is now trimmed. arc connects l_bot end to l_wall_temp start (shifted).
            # The segment of l_wall_temp from 'tangent point' to 'p_wall_low' remains. We assume we don't want a "tail".
            # We want the profile to close along the wall from the arc end to the top.
            # The 'arc' ends at the wall tangent.
            # We delete l_wall_temp.
            # We create l_back from arc.endSketchPoint (or start, depending on geometry) to p_top_back.
            
            # Identify the point on the wall. The arc connects approximate l_bot (slope) to l_wall_temp (vertical).
            # One endpoint of arc is on the wall vertical line.
            # We can check X coordinate in sketch space? Sketch X = Model Y = y_back.
            # Sketch Y = Model Z.
            # Check p_top_back (y=y_back).
            
            p1 = arc.startSketchPoint
            p2 = arc.endSketchPoint
            # The wall point is the one with the LARGER X (closer to back/y_back).
            p_wall_tan = p1 if p1.geometry.x > p2.geometry.x else p2
            
            sp_top_back = tabSketch.modelToSketchSpace(p_top_back)
            
            l_wall_temp.deleteMe()
            
            # 4. Create Back Line
            # We connect p_wall_tan to p_top_back. 
            # Note: p_top_back is at y_back. p_wall_tan is at y_back_shifted.
            # This creates a tiny slope on the back wall of the tab.
            l_back = tabSketchLine.addByTwoPoints(p_wall_tan, sp_top_back)
            
            # Constraint Chain needs update because l_bot end changed and l_back start changed.
            constraints.addCoincident(l_top.endSketchPoint, l_front.startSketchPoint)
            constraints.addCoincident(l_front.endSketchPoint, l_bot.startSketchPoint)
            # constraints.addCoincident(l_bot.endSketchPoint, arc.generatedPoint) -> Not needed, addFillet handles this.
            
            # Make sure l_back (Wall) is tangent to the arc (Smooth transition)
            constraints.addTangent(l_back, arc)
            
            # Close the loop at the top
            constraints.addCoincident(l_back.endSketchPoint, l_top.startSketchPoint)
            
        else:
            l_back = tabSketchLine.addByTwoPoints(tabSketch.modelToSketchSpace(p_bot_back), tabSketch.modelToSketchSpace(p_top_back))
            
            constraints.addCoincident(l_top.endSketchPoint, l_front.startSketchPoint)
            constraints.addCoincident(l_front.endSketchPoint, l_bot.startSketchPoint)
            constraints.addCoincident(l_bot.endSketchPoint, l_back.startSketchPoint)
            constraints.addCoincident(l_back.endSketchPoint, l_top.startSketchPoint)
        # Let's verify standard Orientation. YZ plane. U=Y, V=Z.
        # Horizontal constraint in Fusion sketch usually means parallel to X axis of sketch.
        # Sketch X axis usually corresponds to Model Y for YZ plane.
        

        
        # l_back (wall) should be vertical (Along Z/V, Perpendicular to Y/U)

        
        # Dimensions
        # Width (Top edge length)
        dimensions.addDistanceDimension(l_top.startSketchPoint, l_top.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, l_top.startSketchPoint.geometry, True)
        
        # Root Thickness (Back edge length)
        dimensions.addDistanceDimension(l_back.startSketchPoint, l_back.endSketchPoint, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, l_back.startSketchPoint.geometry, True)
        
        # Tip Thickness (Front edge length)
        dimensions.addDistanceDimension(l_front.startSketchPoint, l_front.endSketchPoint, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, l_front.startSketchPoint.geometry, True)


    else:
        # Angle based (Default)
        actualTabWidth = input.width + BIN_TAB_EDGE_FILLET_RADIUS / math.tan((math.radians(90) - input.overhangAngle) / 2)
        actualTabHeight = actualTabWidth / math.tan(input.overhangAngle)
        line1 = tabSketchLine.addByTwoPoints(
            tabSketch.modelToSketchSpace(adsk.core.Point3D.create(input.origin.x, input.origin.y, tabTopEdgeHeight)),
            tabSketch.modelToSketchSpace(adsk.core.Point3D.create(input.origin.x, input.origin.y, tabTopEdgeHeight - actualTabHeight)),
        )
        line2 = tabSketchLine.addByTwoPoints(
            tabSketch.modelToSketchSpace(adsk.core.Point3D.create(input.origin.x, input.origin.y, tabTopEdgeHeight)),
            tabSketch.modelToSketchSpace(adsk.core.Point3D.create(input.origin.x, input.origin.y - actualTabWidth, tabTopEdgeHeight)),
        )
        line3 = tabSketchLine.addByTwoPoints(
            tabSketch.modelToSketchSpace(adsk.core.Point3D.create(input.origin.x, input.origin.y, tabTopEdgeHeight - actualTabHeight)),
            tabSketch.modelToSketchSpace(adsk.core.Point3D.create(input.origin.x, input.origin.y - actualTabWidth, tabTopEdgeHeight)),
        )

        # horizontal/vertical relative to local sketch XY coordinates
        constraints.addHorizontal(line1)
        constraints.addVertical(line2)
        constraints.addCoincident(line1.startSketchPoint, line2.startSketchPoint)
        constraints.addCoincident(line2.endSketchPoint, line3.endSketchPoint)
        constraints.addCoincident(line1.endSketchPoint, line3.startSketchPoint)

        dimensions.addDistanceDimension(
            tabSketch.originPoint,
            line1.startSketchPoint,
            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
            line1.startSketchPoint.geometry,
            True
            )

        dimensions.addDistanceDimension(
            tabSketch.originPoint,
            line1.startSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            line1.startSketchPoint.geometry,
            True
            )

        dimensions.addDistanceDimension(
            line2.startSketchPoint,
            line2.endSketchPoint,
            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
            line2.endSketchPoint.geometry,
            True
            )
                
        dimensions.addAngularDimension(
            line1,
            line3,
            line1.endSketchPoint.geometry,
            True,
            )

    tabExtrudeFeature = extrudeUtils.simpleDistanceExtrude(
        tabSketch.profiles.item(0),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        input.length,
        adsk.fusion.ExtentDirections.PositiveExtentDirection,
        [],
        targetComponent,
    )
    tabBody = tabExtrudeFeature.bodies.item(0)
    tabBody.name = 'label tab'

    # Fillet logic
    # User wants both Top and Bottom edges of the tip to be filleted.
    # We find all X-collinear edges.
    x_edges = [edge for edge in tabBody.edges if geometryUtils.isCollinearToX(edge)]
    
    # Find the minimum Y (Front of the tab)
    if x_edges:
        min_y = min(x_edges, key=lambda e: e.boundingBox.minPoint.y).boundingBox.minPoint.y
        # Select all edges at that min_y (within float tolerance)
        front_edges = [edge for edge in x_edges if abs(edge.boundingBox.minPoint.y - min_y) < 0.001]
        
        if len(front_edges) >= 2:
            # Sort by Z (height) desc. Top edge is first.
            front_edges_sorted = sorted(front_edges, key=lambda e: e.boundingBox.minPoint.z, reverse=True)
            top_edge = front_edges_sorted[0]
            bottom_edge = front_edges_sorted[1]
            
            # Apply Top Fillet
            if input.tabFilletTop > 0.001:
                filletTop = filletUtils.createFillet(
                    [top_edge],
                    input.tabFilletTop,
                    False,
                    targetComponent
                )
                filletTop.name = 'label tab fillet top'

            # Apply Bottom Fillet
            if input.tabFilletBottom > 0.001:
                filletBottom = filletUtils.createFillet(
                    [bottom_edge],
                    input.tabFilletBottom,
                    False,
                    targetComponent
                )
                filletBottom.name = 'label tab fillet bottom'
        elif len(front_edges) == 1:
            # Maybe Angle method makes a sharp tip with only one edge?
            # If so, just apply top fillet? Or maybe the user sees it as one edge.
            # Let's apply Top radius.
             if input.tabFilletTop > 0.001:
                fillet = filletUtils.createFillet(
                    front_edges,
                    input.tabFilletTop,
                    False,
                    targetComponent
                )
                fillet.name = 'label tab fillet'

    return tabBody
