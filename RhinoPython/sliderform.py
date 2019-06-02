import System.Drawing
import System.Windows.Forms

from System.Drawing import *
from System.Windows.Forms import *

class SliderForm(Form):
    def __init__(self):
        self.__value_changed_callback = None
        self.InitializeComponent()
        self.__disable_events = False
        self._trackBar1.Value = 10
        self._trackBar2.Value = 10
    
    def InitializeComponent(self):
        self._btnOk = System.Windows.Forms.Button()
        self._btnCancel = System.Windows.Forms.Button()
        self._lbl1 = System.Windows.Forms.Label()
        self._trackBar1 = System.Windows.Forms.TrackBar()
        self._txtBox1 = System.Windows.Forms.TextBox()
        self._trackBar2 = System.Windows.Forms.TrackBar()
        self._lbl2 = System.Windows.Forms.Label()
        self._txtBox2 = System.Windows.Forms.TextBox()
        self._trackBar1.BeginInit()
        self._trackBar2.BeginInit()
        self.SuspendLayout()
        # 
        # btnOk
        # 
        self._btnOk.Anchor = System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right
        self._btnOk.DialogResult = System.Windows.Forms.DialogResult.OK
        self._btnOk.Location = System.Drawing.Point(202, 98)
        self._btnOk.Name = "btnOk"
        self._btnOk.Size = System.Drawing.Size(75, 23)
        self._btnOk.TabIndex = 6
        self._btnOk.Text = "OK"
        self._btnOk.UseVisualStyleBackColor = True
        # 
        # btnCancel
        # 
        self._btnCancel.Anchor = System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right
        self._btnCancel.DialogResult = System.Windows.Forms.DialogResult.Cancel
        self._btnCancel.Location = System.Drawing.Point(283, 97)
        self._btnCancel.Name = "btnCancel"
        self._btnCancel.Size = System.Drawing.Size(75, 23)
        self._btnCancel.TabIndex = 7
        self._btnCancel.Text = "Cancel"
        self._btnCancel.UseVisualStyleBackColor = True
        # 
        # lbl1
        # 
        self._lbl1.Location = System.Drawing.Point(12, 15)
        self._lbl1.Name = "lbl1"
        self._lbl1.Size = System.Drawing.Size(69, 23)
        self._lbl1.TabIndex = 0
        self._lbl1.Text = "Ratio (even)"
        # 
        # trackBar1
        # 
        self._trackBar1.Anchor = System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left | System.Windows.Forms.AnchorStyles.Right
        self._trackBar1.AutoSize = False
        self._trackBar1.Location = System.Drawing.Point(87, 12)
        self._trackBar1.Maximum = 100
        self._trackBar1.Name = "trackBar1"
        self._trackBar1.Size = System.Drawing.Size(223, 30)
        self._trackBar1.TabIndex = 1
        self._trackBar1.TickStyle = System.Windows.Forms.TickStyle.None
        self._trackBar1.ValueChanged += self.OnSliderValueChanged
        # 
        # txtBox1
        # 
        self._txtBox1.Anchor = System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right
        self._txtBox1.Location = System.Drawing.Point(316, 12)
        self._txtBox1.Name = "txtBox1"
        self._txtBox1.Size = System.Drawing.Size(42, 20)
        self._txtBox1.TabIndex = 2
        self._txtBox1.TextChanged += self.OnTextBoxChanged
        # 
        # trackBar2
        # 
        self._trackBar2.Anchor = System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left | System.Windows.Forms.AnchorStyles.Right
        self._trackBar2.AutoSize = False
        self._trackBar2.Location = System.Drawing.Point(87, 54)
        self._trackBar2.Maximum = 100
        self._trackBar2.Name = "trackBar2"
        self._trackBar2.Size = System.Drawing.Size(223, 30)
        self._trackBar2.TabIndex = 5
        self._trackBar2.TickStyle = System.Windows.Forms.TickStyle.None
        self._trackBar2.ValueChanged += self.OnSliderValueChanged
        # 
        # lbl2
        # 
        self._lbl2.Location = System.Drawing.Point(12, 57)
        self._lbl2.Name = "lbl2"
        self._lbl2.Size = System.Drawing.Size(69, 23)
        self._lbl2.TabIndex = 3
        self._lbl2.Text = "Ratio (odd)"
        # 
        # m_txtBox2
        # 
        self._txtBox2.Anchor = System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right
        self._txtBox2.Location = System.Drawing.Point(316, 54)
        self._txtBox2.Name = "txtBox2"
        self._txtBox2.Size = System.Drawing.Size(42, 20)
        self._txtBox2.TabIndex = 4
        self._txtBox2.TextChanged += self.OnTextBoxChanged
        # 
        # SliderForm
        # 
        self.AcceptButton = self._btnOk
        self.CancelButton = self._btnCancel
        self.ClientSize = System.Drawing.Size(370, 132)
        self.Controls.Add(self._lbl1)
        self.Controls.Add(self._trackBar1)
        self.Controls.Add(self._txtBox1)
        self.Controls.Add(self._lbl2)
        self.Controls.Add(self._trackBar2)
        self.Controls.Add(self._txtBox2)
        self.Controls.Add(self._btnOk)
        self.Controls.Add(self._btnCancel)
        self.MaximizeBox = False
        self.MinimizeBox = False
        self.MinimumSize = System.Drawing.Size(300, 170)
        self.Name = "SliderForm"
        self.ShowInTaskbar = False
        self.Text = "Slider Form"
        self._trackBar1.EndInit()
        self._trackBar2.EndInit()
        self.ResumeLayout(False)
        self.PerformLayout()

    def OnSliderValueChanged(self, sender, e):
        #__disable_events is used to keep from circular
        #events being called
        if( self.__disable_events!=True ):
            self.__disable_events = True
            self._txtBox1.Text = str(self._trackBar1.Value/10.0)
            self._txtBox2.Text = str(self._trackBar2.Value/10.0)
            self.__disable_events = False
        if( self.__value_changed_callback!=None ):
            val_even = self._trackBar1.Value/10.0
            val_odd = self._trackBar2.Value/10.0
            self.__value_changed_callback(val_even, val_odd)

    def SetValueChangedCallback(self, callback_object):
        self.__value_changed_callback = callback_object

    def OnTextBoxChanged(self, sender, e):
        #__disable_events is used to keep from circular
        #events being called
        if( self.__disable_events!=True ):
          self.__disable_events = True
          try:
              self._trackBar1.Value = float(self._txtBox1.Text) * 10.0
              self._trackBar2.Value = float(self._txtBox2.Text) * 10.0
          except:
              self._txtBox1.Text = str(self._trackBar1.Value/10.0)
              self._txtBox2.Text = str(self._trackBar2.Value/10.0)
          self.__disable_events = False