/*
* 
* An XmlReader implementation for loading comma delimited files (.csv files)
*
* Copyright (c) 2001-2005 Microsoft Corporation. All rights reserved.
*
* Chris Lovett
* 
*/

using System;
using System.Xml;
using System.IO;
using System.Collections;
using System.Text;
using System.Net;

namespace Microsoft.Xml {
    enum State {
        Initial,
        Root,
        Row,
        Field,
        FieldValue,
        EndField,
        EndRow,
        Attr,
        AttrValue,
        EndRoot,
        Eof
    }


    /// <summary>
    /// Summary description for XmlCsvReader.
    /// </summary>
    public class XmlCsvReader : XmlReader {
        CsvReader _csvReader;
        Uri _baseUri;
        Uri _href;
        string _root = "root";
        string _rowname = "row";
        XmlNameTable _nt;
        string[] _names;
        State _state = State.Initial;
        int _attr = 0;
        bool _asAttrs = false;
        bool _firstRowHasColumnNames = false;
        char _delimiter;
        string _proxy;
        Encoding _encoding;

        protected override void Dispose(bool disposing)
        {
            base.Dispose(disposing);
            if (_csvReader != null)
            {
                _csvReader.Close();
            }
        }

        void Init() {
            _state = State.Initial;
            _attr = 0;
        }

        /// <summary>
        /// Construct XmlCsvReader.  You must specify an HRef
        /// location or a TextReader before calling Read().
        /// </summary>
        public XmlCsvReader() {
            _nt = new NameTable();
            _encoding = Encoding.UTF8;
        }

        /// <summary>
        /// Construct an XmlCsvReader.
        /// </summary>
        /// <param name="input">The .csv input stream</param>
        /// <param name="baseUri">The base URI of the .csv.</param>
        /// <param name="nametable">The nametable to use for atomizing element names</param>
        public XmlCsvReader(Stream input, Encoding encoding, Uri baseUri, XmlNameTable nametable) {
            _baseUri = baseUri;
            _encoding = encoding;
            _csvReader = new CsvReader(input, encoding, 4096);
            _nt = nametable;
        }

        /// <summary>
        /// Construct an XmlCsvReader.
        /// </summary>
        /// <param name="input">The .csv input text reader</param>
        /// <param name="baseUri">The base URI of the .csv.</param>
        /// <param name="nametable">The nametable to use for atomizing element names</param>
        public XmlCsvReader(TextReader input, Uri baseUri, XmlNameTable nametable) {
            _baseUri = baseUri;
            _encoding = Encoding.UTF8;
            _csvReader = new CsvReader(input, 4096);
            _nt = nametable;
        }

        /// <summary>
        /// Specifies the base URI to use for resolving the Href property.
        /// This is optional.
        /// </summary>
        public string BaseUri {
            get { return _baseUri == null ? "" : _baseUri.AbsoluteUri; }
            set { _baseUri = new Uri(value); }
        }
        
        /// <summary>
        /// Specifies the encoding to use when loading the .csv file.
        /// </summary>
        public Encoding Encoding {
            get { return _encoding == null ? System.Text.Encoding.UTF8 : _encoding; }
            set { _encoding = value; }
        }

        /// <summary>
        /// Specifies the URI location of the .csv file to parse.
        /// This can also be a local file name.
        /// This can be a relative URI if a BaseUri has been provided.
        /// You must specify either this property or a TextReader as input
        /// before calling Read.
        /// </summary>
        public string Href {
            get { return _href == null ? "" : _href.AbsoluteUri; }
            set { 
                if (_baseUri != null) {
                    _href = new Uri(_baseUri, value); 
                } else {
                    try {
                        _href = new Uri(value); 
                    } 
                    catch (Exception) {
                        string file = Path.GetFullPath(value);
                        _href = new Uri(file);
                    } 
                    _baseUri = _href;
                }
                _csvReader = null;
                Init();
            }
        }

        /// <summary>
        /// Specifies the proxy server.  This is only needed for internet HTTP requests
        /// where the caller is behind a proxy server internet gateway. 
        /// </summary>
        public string Proxy {
            get { return _proxy; }
            set { _proxy = value; }
        }

        /// <summary>
        /// Returns the TextReader that contains the .csv file contents.
        /// </summary>
        public TextReader TextReader {
            get { return _csvReader == null ? null : _csvReader.Reader; }
            set { _csvReader = new CsvReader(value, 4096);
                _csvReader.Delimiter = this.Delimiter;
                Init(); }
        }

        /// <summary>
        /// Specifies the name of the root element, the default is "root".
        /// </summary>
        public string RootName {
            get { return _root; }
            set { _root = _nt.Add(value); }
        }

        /// <summary>
        /// Specifies the name of the XML element generated for each row
        /// of .csv data.  The default is "row".
        /// </summary>
        public string RowName {
            get { return _rowname; }
            set {  _rowname = _nt.Add(value); }
        }

        /// <summary>
        /// Specifies whether the first row contains column names.
        /// Default is false.
        /// </summary>
        public bool FirstRowHasColumnNames {
            get { return _firstRowHasColumnNames; }
            set { _firstRowHasColumnNames = value;  }
        }

        /// <summary>
        /// Specifies whether to return the columns as attributes
        /// or as child elements.  Default is false.
        /// </summary>
        public bool ColumnsAsAttributes {
            get { return _asAttrs; }
            set { _asAttrs = value;  }
        }

        /// <summary>
        /// Instead of reading the column names from the stream you can also
        /// provide the column names yourself.
        /// </summary>
        public string[] ColumnNames {
            get { return _names; }
            set { 
                // atomize the names.
                ArrayList copy = new ArrayList();
                for (int i = 0; i < value.Length; i++) {
                    copy.Add(_nt.Add(value[i]));
                }
                _names = (string[])copy.ToArray(typeof(string));
            }
        }

        /// <summary>
        /// Gets or sets the column delimiter.  Default is '\0' which means 
        /// the reader will auto detect the delimiter.
        /// </summary>
        public char Delimiter {
            get { return _delimiter; }
            set { _delimiter = value; if (_csvReader != null) _csvReader.Delimiter = value;}
        }

        void ReadColumnNames() {
            if (_csvReader.Read()) {
                // If column names were already provided then we just skip this row.
                if (_names == null) {
                    _names = new string[_csvReader.FieldCount];
                    for (int i = 0; i < _csvReader.FieldCount; i++) {
                        _names[i] = _nt.Add(_csvReader[i]);
                    }
                }
            }
        }

        public override XmlNodeType NodeType { 
            get {
                switch (_state) {
                    case State.Initial: 
                    case State.Eof:
                        return XmlNodeType.None;
                    case State.Root:
                    case State.Row:
                    case State.Field:
                        return XmlNodeType.Element;
                    case State.Attr:
                        return XmlNodeType.Attribute;
                    case State.AttrValue:
                    case State.FieldValue:
                        return XmlNodeType.Text;
                    default:
                        return XmlNodeType.EndElement;
                }       
            }
        }

        public override string Name {
            get {
                return this.LocalName;
            }
        }

        public override string LocalName { 
            get {
                switch (_state) {
                    case State.Attr:
                    case State.Field:
                    case State.EndField:
                        if (_names == null || _attr >= _names.Length) {
                            return this._nt.Add("a"+_attr);
                        } 
                        return XmlConvert.EncodeLocalName(_names[_attr]);
                    case State.Root:
                    case State.EndRoot:
                        return _root;
                    case State.Row:
                    case State.EndRow:
                        return _rowname;
                }
                return string.Empty;
            }
        }

        public override string NamespaceURI { 
            get {
                return String.Empty;
            }
        }

        public override string Prefix { 
            get {
                return String.Empty;
            }
        }

        public override bool HasValue { 
            get {
                if (_state == State.Attr || _state == State.AttrValue || _state == State.FieldValue) {
                    return Value != String.Empty;
                }
                return false;
            }
        }

        public override string Value { 
            get {
                if (_state == State.Attr || _state == State.AttrValue || _state == State.FieldValue) {
                    return _csvReader[_attr];
                }
                return null;
            }
        }

        public override int Depth { 
            get {
                switch (_state) {
                    case State.Row:
                    case State.EndRow:
                        return 1;
                    case State.Attr:
                    case State.Field:
                    case State.EndField:
                        return 2;
                    case State.AttrValue:
                    case State.FieldValue:
                        return 3;
                }       
                return 0;
            }
        }

        public override string BaseURI { 
            get {
                return _baseUri.AbsolutePath;
            }
        }

        public override bool IsEmptyElement { 
            get {
                if (_state == State.Row && _asAttrs) 
                    return true;

                if (_state == State.Field && _csvReader[_attr] == String.Empty) 
                    return true;

                return false;
            }
        }
        public override bool IsDefault { 
            get {
                return false;
            }
        }
        public override char QuoteChar { 
            get {
                return _csvReader.QuoteChar;
            }
        }

        public override XmlSpace XmlSpace { 
            get {
                return XmlSpace.Default;
            }
        }

        public override string XmlLang { 
            get {
                return String.Empty;
            }
        }

        public override int AttributeCount { 
            get {
                if (! _asAttrs) return 0;

                if (_state == State.Row || _state == State.Attr || _state == State.AttrValue) {
                    return _csvReader.FieldCount;
                }
                return 0;
            }
        }

        public override string GetAttribute(string name) {
            if (! _asAttrs) return null;

            if (_state == State.Row || _state == State.Attr || _state == State.AttrValue) {
                int i = GetOrdinal(name);
                if (i >= 0) 
                    return GetAttribute(i);
            }
            return null;
        }

        int GetOrdinal(string name) {
            if (_names != null) {
                string n = _nt.Add(name);
                for (int i = 0; i < _names.Length; i++) {
                    if ((object)_names[i] == (object)n)
                        return i;
                }
                throw new Exception("Attribute '"+name+"' not found.");
            }
            // names are assigned a0, a1, a2, ...
            return Int32.Parse(name.Substring(1));
        }

        public override string GetAttribute(string name, string namespaceURI) {
            if (namespaceURI != string.Empty && namespaceURI != null) return null;
            return GetAttribute(name);
        }

        public override string GetAttribute(int i) {
            if (! _asAttrs) return null;
            if (_state == State.Row || _state == State.Attr || _state == State.AttrValue) {
                return _csvReader[i];
            }
            return null;
        }

        public override string this [ int i ] { 
            get {
                return GetAttribute(i);
            }
        }

        public override string this [ string name ] { 
            get {
                return GetAttribute(name);
            }
        }

        public override string this [ string name,string namespaceURI ] { 
            get {
                return GetAttribute(name, namespaceURI);
            }
        }

        public override bool MoveToAttribute(string name) {
            if (! _asAttrs) return false;
            if (_state == State.Row || _state == State.Attr || _state == State.AttrValue) {
                int i = GetOrdinal(name);
                if (i < 0) return false;
                MoveToAttribute(i);
            }
            return false;
        }

        public override bool MoveToAttribute(string name, string ns) {
            if (ns != string.Empty && ns != null) return false;
            return MoveToAttribute(name);
        }

        public override void MoveToAttribute(int i) {
            if (_asAttrs) {
                if (_state == State.Row || _state == State.Attr || _state == State.AttrValue) {
                    _state = State.Attr;
                    _attr = i;
                }     
            }
        }

        public override bool MoveToFirstAttribute() {
            if (! _asAttrs) return false;
            if (AttributeCount > 0) {
                _attr = 0;
                _state = State.Attr;
                return true;
            }
            return false;
        }

        public override bool MoveToNextAttribute() {
            if (! _asAttrs) return false;
            if (_attr < AttributeCount-1) {
                _attr = (_state == State.Attr || _state == State.AttrValue) ? _attr+1 : 0;
                _state = State.Attr;
                return true;
            }
            return false;
        }

        public override bool MoveToElement() {
            if (! _asAttrs) return true;

            if (_state == State.Root || _state == State.EndRoot || _state == State.Row) {
                return true;
            }
            else if (_state == State.Attr || _state == State.AttrValue) {
                _state = State.Row;
                return true;
            }                               
            return false;
        }

        public override bool Read() {
            switch (_state) {
                case State.Initial:
                    if (_csvReader == null) {
                        throw new Exception("You must provide an input location via the Href property, or provide an input stream via the TextReader property.");
                    }
                    if (_firstRowHasColumnNames) {
                        ReadColumnNames();
                    }
                    _state = State.Root;
                    return true;
                case State.Eof:
                    return false;
                case State.Root:
                case State.EndRow:          
                    if (_csvReader.Read()) {
                        _state = State.Row;
                        return true;
                    }
                    _state = State.EndRoot;
                    return true;        
                case State.EndRoot:
                    _state = State.Eof;
                    return false;
                case State.Row:
                    if (_asAttrs) {
                        _attr = 0;
                        goto case State.EndRow;
                    } 
                    else {
                        _state = State.Field;
                        _attr = 0;
                        return true;
                    }
                case State.Field:
                    if (!IsEmptyElement) {
                        _state = State.FieldValue;
                    } 
                    else {
                        goto case State.EndField;
                    }
                    return true;
                case State.FieldValue:
                    _state = State.EndField;
                    return true;
                case State.EndField:
                    if (_attr < _csvReader.FieldCount-1) {
                        _attr++;
                        _state = State.Field;
                        return true;
                    }
                    _state = State.EndRow;
                    return true;
                case State.Attr:
                case State.AttrValue:
                    _state = State.Root;
                    _attr = 0;
                    goto case State.Root;
            }
            return false;
        }

        public override bool EOF { 
            get {
                return _state == State.Eof;
            }
        }

        public override ReadState ReadState { 
            get {
                if (_state == State.Initial) return ReadState.Initial;
                else if (_state == State.Eof) return ReadState.EndOfFile;
                return ReadState.Interactive;
            }
        }

        public override string ReadContentAsString()
        {
            if (_state == State.AttrValue || _state == State.Attr) {
                return _csvReader[_attr];
            }
            return String.Empty;
        }

        public override string ReadInnerXml() {
            StringWriter sw = new StringWriter();
            XmlWriterSettings ws = new XmlWriterSettings();
            ws.Indent = true;
            using (XmlWriter xw = XmlWriter.Create(sw, ws))
            {
                while (!this.EOF && this.NodeType != XmlNodeType.EndElement)
                {
                    xw.WriteNode(this, true);
                }
            }
            return sw.ToString();
        }

        public override string ReadOuterXml() {
            StringWriter sw = new StringWriter();

            XmlWriterSettings ws = new XmlWriterSettings();
            ws.Indent = true;
            using (XmlWriter xw = XmlWriter.Create(sw, ws))
            {
                xw.WriteNode(this, true);
            }

            return sw.ToString();
        }

        public override XmlNameTable NameTable { 
            get {
                return _nt;
            }
        }

        public override string LookupNamespace(string prefix) {     
            return null;
        }

        public override void ResolveEntity() {
            throw new NotImplementedException();
        }

        public override bool ReadAttributeValue() {
            if (_state == State.Attr) {
                _state = State.AttrValue;
                return true;
            }
            else if (_state == State.AttrValue) {
                return false;
            }
            throw new Exception("Not on an attribute.");
        } 

    }

    class CsvReader {
        TextReader _r;
        char[] _buffer;
        int _pos;
        int _used;

        // assumes end of record delimiter is {CR}{LF}, {CR}, or {LF}
        // possible values are {CR}{LF}, {CR}, {LF}, ';', ',', '\t'
        // char _recDelim;

        char _colDelim; // possible values ',', ';', '\t', '|'
        char _quoteChar;

        ArrayList _values;
        int _fields;

        public CsvReader(Stream stm, Encoding encoding, int bufsize) {  // the location of the .csv file
            _r = new StreamReader(stm, encoding, true);
            _buffer = new char[bufsize];
            _values = new ArrayList();
        }     
        public CsvReader(TextReader stm, int bufsize) {  // the location of the .csv file
            _r = stm;
            _buffer = new char[bufsize];
            _values = new ArrayList();
        }     

        public TextReader Reader {
            get { return _r; }
        }

        const int EOF = 0xffff;

        public bool Read() { // read a record.
            _fields = 0;
            char ch = ReadChar();
            if (ch == 0) return false;
            while (ch != 0 && ch == '\r' || ch == '\n' || ch == ' ') 
                ch = ReadChar();
            if (ch == 0) return false;

            while (ch != 0 && ch != '\r' && ch != '\n') {
                StringBuilder sb = AddField();
                if (ch == '\'' || ch == '"') {
                    _quoteChar= ch;         
                    char c = ReadChar();
                    bool done = false;
                    while (!done && c != 0) {
                        while (c != 0 && c != ch) { // scan literal.
                            sb.Append(c);
                            c = ReadChar();
                        }
                        if (c == ch) {
                            done = true;
                            char next = ReadChar(); // consume end quote
                            if (next == ch ) {
                                // it was an escaped quote sequence "" inside the literal
                                // so append a single " and consume the second end quote.
                                done = false;
                                sb.Append(next);
                                c = ReadChar();
                                if (_colDelim != 0 && c == _colDelim){
                                    // bad form, but this is probably a record separator.
                                    done = true;
                                }
                            } else if (_colDelim != 0 && next != _colDelim && next != 0 && next != ' ' && next != '\n' && next != '\r') {
                                // it was an un-escaped quote embedded inside a string literal
                                // in this case the quote is probably just part of the text so ignore it.
                                done = false;
                                sb.Append(c);
                                sb.Append(next);
                                c = ReadChar();
                            } else {
                                c = next;
                            }
                        }
                    }
                    ch = c;         
                } 
                else {        
                    // skip whitespace
                    while (ch == ' ')
                    {
                        ch = ReadChar();
                    }
                    // scan number, date, time, float, etc.
                    while (ch != 0 && ch != '\n' && ch != '\r') {
                        if (ch == _colDelim || (_colDelim == '\0' && (ch == ',' || ch == ';' || ch == '\t' || ch == '|')))
                            break;
                        sb.Append(ch);
                        ch = ReadChar();
                    }
                } 
                if (ch == _colDelim || (_colDelim == '\0' && (ch == ',' || ch == ';' || ch == '\t' || ch == '|'))){
                    _colDelim = ch;
                    ch = ReadChar();
                    if (ch == '\n' || ch == '\r') {
                        sb=AddField(); // blank field.
                    }
                }
            }
            return true;
        }

        public char QuoteChar { get { return _quoteChar; } }
        public char Delimiter { get { return _colDelim; } set { _colDelim = value ; }}

        public int FieldCount { get { return _fields; } }

        public string this[int i] { get { return ((StringBuilder)_values[i]).ToString(); } }

        char ReadChar() {
            if (_pos == _used) {
                _pos = 0;
                _used = _r.Read(_buffer, 0, _buffer.Length);
            }
            if (_pos == _used) {
                return (char)0;
            }
            return _buffer[_pos++];
        }

        StringBuilder AddField() {
            if (_fields == _values.Count) {
                _values.Add(new StringBuilder());
            }
            StringBuilder sb = (StringBuilder)_values[_fields++];
            sb.Length = 0;
            return sb;
        }

        

        public void Close() {
            _r.Dispose();       
        }
    }
  
}