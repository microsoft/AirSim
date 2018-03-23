using LogViewer.Utilities;
using Microsoft.Networking.Mavlink;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace LogViewer.Controls
{
    /// <summary>
    /// Interaction logic for Console.xaml
    /// </summary>
    public partial class Console : UserControl
    {
        static SolidColorBrush ConsoleTextBrush = new SolidColorBrush(Colors.LimeGreen);

        public Console()
        {
            InitializeComponent();

            CommandManager.AddPreviewExecutedHandler(ConsoleTextBox, new ExecutedRoutedEventHandler(OnPreviewCommand));
        }

        private void OnPreviewCommand(object sender, ExecutedRoutedEventArgs e)
        {
            var property = e.Command.GetType().GetProperty("Name");
            if (property != null && property.PropertyType == typeof(string))
            {
                string name = (string)property.GetValue(e.Command);
                if (name == "Paste")
                {
                    HandlePasteCommand();
                    e.Handled = true;
                }
            }
        }

        private void HandlePasteCommand()
        {
            string plainText = Clipboard.GetText(TextDataFormat.UnicodeText);

            // make sure paste lands at the end
            ConsoleTextBox.Selection.Select(ConsoleTextBox.Document.ContentEnd, ConsoleTextBox.Document.ContentEnd);
            ConsoleTextBox.ScrollToEnd();
            AddUserText(plainText);
        }

        public  void Clear()
        {
            ConsoleTextBox.Document.Blocks.Clear();
        }

        public MavlinkChannel Channel { get; set; }


        public void Write(string text)
        {
            var doc = ConsoleTextBox.Document;
            // todo: process console navigation commands...
            int len = text.Length;
            if (len > 3 && text[len - 1] == 'K' && text[len - 2] == '[' && text[len - 3] == '\x1b')
            {
                // this is an ERASE_END_LINE command which we ignore.
                text = text.Substring(0, len - 3);
            }            
            Paragraph last = doc.Blocks.LastBlock as Paragraph;
            if (last == null)
            {
                last = new Paragraph();
                doc.Blocks.Add(last);
            }
            // slide the output into a run "just before" the user editing so that user editing always slides to the bottom of the document.
            Run run = last.Inlines.LastInline as Run;
            if (run != null && (string)run.Tag != "user")
            {
                // Ah, user must have deleted everything, so recreate out setup here...
                run.Foreground = ConsoleTextBrush;

                var userRun = new Run(" ") { Tag = "user" };
                last.Inlines.Add(userRun);
            }

            if (run == null)
            {
                // different color stops these two runs from getting merged automatically.
                run = new Run() { Foreground = ConsoleTextBrush };
                last.Inlines.Add(run);

                var userRun = new Run(" ") { Tag = "user" };
                last.Inlines.Add(userRun);
            }

            if ((string)run.Tag == "user")
            {
                var previousrun = run.PreviousInline as Run;
                if (previousrun == null)
                {
                    var newrun = new Run() { Foreground = ConsoleTextBrush };
                    last.Inlines.InsertBefore(run, newrun);
                    run = newrun;
                }
                else
                {
                    run = previousrun;
                }
            }

            // todo: process binary console commands embedded in the text...
            run.Text += text;

            // scroll to end.
            ConsoleTextBox.Selection.Select(ConsoleTextBox.Document.ContentEnd, ConsoleTextBox.Document.ContentEnd);            
            ConsoleTextBox.ScrollToEnd();
        }

        public void Show()
        {
            this.Visibility = Visibility.Visible;
            SendConsoleMessage("\n");
            ConsoleTextBox.Focus();
        }

        public void Hide()
        {
            this.Visibility = Visibility.Collapsed;

            MAVLink.mavlink_serial_control_t ctrl = new MAVLink.mavlink_serial_control_t();
            SendMessage(MAVLink.MAVLINK_MSG_ID.SERIAL_CONTROL, ctrl);
        }

        private void SendConsoleMessage(string msg)
        {
            var channel = this.Channel;
            if (channel != null)
            {
                MAVLink.mavlink_serial_control_t ctrl = new MAVLink.mavlink_serial_control_t();
                ctrl.device = (byte)MAVLink.SERIAL_CONTROL_DEV.SHELL;
                ctrl.flags = (byte)(MAVLink.SERIAL_CONTROL_FLAG.RESPOND | MAVLink.SERIAL_CONTROL_FLAG.EXCLUSIVE);
                ctrl.baudrate = 0;
                ctrl.timeout = 0;
                byte[] ascii = System.Text.Encoding.ASCII.GetBytes(msg);

                int len = ascii.Length;
                if (len > 70) len = 70;
                ctrl.count = (byte)len;

                byte[] copy = new byte[70];
                Array.Copy(ascii, copy, len);
                ctrl.data = copy;

                SendMessage(MAVLink.MAVLINK_MSG_ID.SERIAL_CONTROL, ctrl);
            }
        }

        void SendMessage(MAVLink.MAVLINK_MSG_ID id, object mavlinkPayload)
        {
            var channel = this.Channel;
            if (channel != null)
            {
                MavLinkMessage message = new MavLinkMessage();
                message.ComponentId = (byte)MAVLink.MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER;
                message.SystemId = 255;
                message.MsgId = id;
                message.TypedPayload = mavlinkPayload;
                try
                {
                    channel.SendMessage(message);
                }
                catch (Exception ex)
                {
                    Write(ex.Message);
                }
            }
        }

        private void OnConsoleKeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                // get the current line of text after all their editing is finished and send it to the drone.
                var doc = ConsoleTextBox.Document;
                // since the user editable Run is unique, we can get just that text easily.

                string text = SetUserText("");
                if (text == "clear" || text == "cls")
                {
                    // handle this one inline.
                    doc.Blocks.Clear();
                    SendConsoleMessage("\n");
                    return;
                }
                if (pos < history.Count && history[pos] == text)
                {
                    // then we are replaying history, so rewrite history 
                }
                else
                {
                    history.Add(text);
                    pos = history.Count;
                }
                text += "\n";

                e.Handled = true; // don't let newline go to RichTextBox because this will split our "user" text Run.
                SendConsoleMessage(text);
            }
            else if (e.Key == Key.Back)
            {
                // don't let RichTextBox handle this delete because it will remove the "user" tagged text run.
                TextPointer selection = ConsoleTextBox.Selection.Start;
                int len = selection.GetTextRunLength(LogicalDirection.Backward);
                if (len > 1)
                {
                    // remove the text 
                    selection.GetPositionAtOffset(-1).DeleteTextInRun(1);
                }
                e.Handled = true;
            }
            else if (e.Key == Key.Up)
            {
                if (pos > 0 && pos - 1 < history.Count)
                {
                    pos--;
                    SetUserText(history[pos]);
                }
                e.Handled = true;
            }
            else if (e.Key == Key.Down)
            {
                if (pos + 1 < history.Count)
                {
                    pos++;
                    SetUserText(history[pos]);
                }
                else
                {
                    pos = history.Count;
                    SetUserText("");
                }
                e.Handled = true;
            }
            else if (e.Key == Key.Up)
            {
                if (pos + 1 < history.Count)
                {
                    pos++;
                    SetUserText(history[pos]);
                }
                else
                {
                    pos = history.Count;
                    SetUserText("");
                }
                e.Handled = true;

            }
        }

        string SetUserText(string newText)
        {
            TextPointer selection = ConsoleTextBox.Selection.Start;
            int len = selection.GetTextRunLength(LogicalDirection.Backward);
            string text = selection.GetTextInRun(LogicalDirection.Backward).Trim();
            // delete the user input (so console can echo it back in green).
            len--; // not including the leading space which we keep to stop the runs from getting merged.
            selection.GetPositionAtOffset(-len).DeleteTextInRun(len);
            selection.InsertTextInRun(newText);
            return text;
        }

        void AddUserText(string newText)
        {
            TextPointer selection = ConsoleTextBox.Selection.Start;
            selection.InsertTextInRun(newText);
        }

        List<string> history = new List<string>();
        int pos = 0;

    }
}
