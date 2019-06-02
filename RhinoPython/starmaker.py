import sliderform #SliderForm defined in SharpDevelop
import rhinoscriptsyntax as rs
import Rhino
import System.Drawing
import scriptcontext
import System.Windows.Forms


class SliderFormController():
    """Hook up UI of the SliderForm with some Rhino specific actions"""

    def __init__(self, curve_ids, dialog):
        # tell the SliderForm about a function to call when the slider values change
        dialog.SetValueChangedCallback(self.UpdateTempCurves)
        # Set up display conduit events to allow drawing of temporary preview
        # geometry while the values are being adjusted by the user
        #
        # CalculateBoundingBox event is used to adjust the view's clipped bounds.
        # Preview geometry may be outside the area of the bounding box of geometry
        # in the document and therefore may be clipped if we don't adjust the bbox
        Rhino.Display.DisplayPipeline.CalculateBoundingBox += self.OnCalcBoundingBox
        # DrawForeground event is used to actually draw some temporary geometry on top
        # of everything else. Depth writing/testing are off at this stage so all
        # geometry is drawn on top
        Rhino.Display.DisplayPipeline.DrawForeground += self.OnDrawForeground
        # Get notified of when the form closes so we can remove our conduits
        dialog.FormClosed += self.OnSliderFormClosed
        
        # temp_curves and temp_curves_bbox are used for dynamic display
        self.__temp_curves = []
        self.__temp_curves_bbox = None
        # original_curves is a list of (id, centroid, points) for each curve
        # provided in the constructor. This is the information we need to
        # quickly construct our temp_curves
        self.__original_curves = []
        for curve_id in curve_ids:
            # curve_id is a Guid. Try to get the Curve geometry from this Guid
            objref = Rhino.DocObjects.ObjRef(curve_id)
            curve_geometry = objref.Curve()
            objref.Dispose()
            if( curve_geometry!=None and curve_geometry.IsClosed ):
                mp = Rhino.Geometry.AreaMassProperties.Compute(curve_geometry)
                rc, points = curve_geometry.TryGetPolyline()
                if( rc ): self.__original_curves.append( (curve_id, mp.Centroid, points) )
    
    def OnSliderFormClosed(self, sender, e):
        # Form has closed. Remove our display conduit
        Rhino.Display.DisplayPipeline.DrawForeground -= self.OnDrawForeground
        Rhino.Display.DisplayPipeline.CalculateBoundingBox -= self.OnCalcBoundingBox
        scriptcontext.doc.Views.Redraw()
    
    def OnDrawForeground(self, sender, e):
        # This function is called while updating viewports. Try not to perform
        # too many calculations in here to minimize display performance hit.
        # Draw each temp_curve in red and a "control" polygon in feedback color
        curve_color = System.Drawing.Color.Red
        feedback_color = Rhino.ApplicationSettings.AppearanceSettings.FeedbackColor
        for points, curve in self.__temp_curves:
            e.Display.DrawCurve(curve, curve_color, 1)
            e.Display.DrawDottedPolyline( points, feedback_color, True )
            e.Display.DrawPoints(points, Rhino.Display.PointStyle.ControlPoint, 2, feedback_color)

    def OnCalcBoundingBox(self, sender, e):
        # Update the boundingbox to include our temporary curves
        if(self.__temp_curves_bbox!=None):
            e.IncludeBoundingBox(self.__temp_curves_bbox)
        
    def UpdateTempCurves(self, ratio_offset_even, ratio_offset_odd):
        self.__temp_curves = []
        self.__temp_curves_bbox=None
        for id, centroid, cvs in self.__original_curves:
            # "cook" up a temp_curve from the original curve data
            temp_points = []
            count = len(cvs)
            for index, cv in enumerate(cvs):
                vector = cv - centroid
                # use %2 for even/odd test
                if( index%2 == 0 or index==(count-1)):
                    vector *= ratio_offset_even
                else:
                    vector *= ratio_offset_odd
                point = centroid + vector
                temp_points.append(point)
            temp_curve = Rhino.Geometry.Curve.CreateControlPointCurve(temp_points, 3)
            if( temp_curve!=None ):
                self.__temp_curves.append((temp_points,temp_curve))
                if( self.__temp_curves_bbox==None ):
                    self.__temp_curves_bbox = temp_curve.GetBoundingBox(False)
                else:
                    self.__temp_curves_bbox = Rhino.Geometry.BoundingBox.Union( self.__temp_curves_bbox, temp_curve.GetBoundingBox(False))
        scriptcontext.doc.Views.Redraw()

    def AddCurvesToDocument(self):
        # add the temp_curves to the document so they become "real" Rhino geometry
        for points, curve in self.__temp_curves:
            scriptcontext.doc.Objects.AddCurve(curve)
        scriptcontext.doc.Views.Redraw()


def main_function():
    """
    main function that runs for this script when it is run as __main__
    Returns: True or False indicating success or failure
    """
    
    # get a list of curves - use a custom filter function to
    # not allow anything but closed polylines
    def onlyclosedpolylines( rhino_object, geometry, component_index ):
        if( isinstance(geometry, Rhino.Geometry.Curve) ):
            if( geometry.IsPolyline() and geometry.IsClsed ): return True
        return False

    curves = rs.GetObjects("pick curves (polylines.planar.closed)", rs.filter.curve, custom_filter = onlyclosedpolylines)
    if( curves!=None ):
        # create the SliderForm and hook a "controller" object to it
        dialog = sliderform.SliderForm()
        controller = SliderFormController(curves, dialog)
    
        # Show our dialog in a semi-modal way.
        if( Rhino.UI.Dialogs.ShowSemiModal( dialog ) == System.Windows.Forms.DialogResult.OK ):
            controller.AddCurvesToDocument()
            return True
    return False


if( __name__ == "__main__" ):
    main_function()