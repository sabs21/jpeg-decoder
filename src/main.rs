#[non_exhaustive]
struct Markers;

impl Markers {
    // ALL MARKERS ARE 2 BYTES (u16)
    // THE FIRST BYTE IS ALWAYS 0xFF (big endian)
    // Start of Frame (SOF), non-differential, Huffman coding
    pub const SOF0:  u8 = 0xc0; // Baseline DCT
    pub const SOF1:  u8 = 0xc1; // Extended Sequential DCT
    pub const SOF2:  u8 = 0xc2; // Progressive DCT
    pub const SOF3:  u8 = 0xc3; // Lossless (sequential)
    // Start of Frame (SOF), differential, Huffman coding
    pub const SOF5:  u8 = 0xc5; // Differential sequential DCT
    pub const SOF6:  u8 = 0xc6; // Differential progressive DCT
    pub const SOF7:  u8 = 0xc7; // Differential lossless (sequential)
    // Start of Frame (SOF), non-differential, arithmetic coding
    pub const JPG:   u8 = 0xc8; // Reserved for JPEG extensions
    pub const SOF9:  u8 = 0xc9; // Extended sequential DCT
    pub const SOF10: u8 = 0xca; // Progressive DCT
    pub const SOF11: u8 = 0xcb; // Lossless (sequential)
    // Start of Frame (SOF), differential, arithmetic coding
    pub const SOF13: u8 = 0xcd; // Differential sequential DCT
    pub const SOF14: u8 = 0xce; // Differential progressive DCT
    pub const SOF15: u8 = 0xcf; // Differential lossless (sequential)
    // Huffman table spec
    pub const DHT:   u8 = 0xc4; // Define Huffman Table(s)
    // Arithmetic coding conditioning spec
    pub const DAC:   u8 = 0xcc; // Define Arithmetic Coding conditioning(s)
    // Restart interval termination
    pub const RST0:  u8 = 0xd0;
    pub const RST1:  u8 = 0xd1;
    pub const RST2:  u8 = 0xd2;
    pub const RST3:  u8 = 0xd3;
    pub const RST4:  u8 = 0xd4;
    pub const RST5:  u8 = 0xd5;
    pub const RST6:  u8 = 0xd6;
    pub const RST7:  u8 = 0xd7;
    pub const SOI:   u8 = 0xd8; // Start of Image
    pub const EOI:   u8 = 0xd9; // End of Image
    pub const SOS:   u8 = 0xda; // Start of Scan
    pub const DQT:   u8 = 0xdb; // Define Quantization Table(s)
    pub const DNL:   u8 = 0xdc; // Define number of lines
    pub const DRI:   u8 = 0xdd; // Define restart interval
    pub const DHP:   u8 = 0xde; // Define hierarchical progression
    pub const EXP:   u8 = 0xdf; // Expand reference component(s)
    // Reserved for application segments
    pub const APP0:  u8 = 0xe0;
    pub const APP1:  u8 = 0xe1;
    pub const APP2:  u8 = 0xe2;
    pub const APP3:  u8 = 0xe3;
    pub const APP4:  u8 = 0xe4;
    pub const APP5:  u8 = 0xe5;
    pub const APP6:  u8 = 0xe6;
    pub const APP7:  u8 = 0xe7;
    pub const APP8:  u8 = 0xe8;
    pub const APP9:  u8 = 0xe9;
    pub const APP10: u8 = 0xea;
    pub const APP11: u8 = 0xeb;
    pub const APP12: u8 = 0xec;
    pub const APP13: u8 = 0xed;
    pub const APP14: u8 = 0xee;
    pub const APP15: u8 = 0xef;
    // Reserved for JPEG extensions
    pub const JPG0:  u8 = 0xf0;
    pub const JPG1:  u8 = 0xf1;
    pub const JPG2:  u8 = 0xf2;
    pub const JPG3:  u8 = 0xf3;
    pub const JPG4:  u8 = 0xf4;
    pub const JPG5:  u8 = 0xf5;
    pub const JPG6:  u8 = 0xf6;
    pub const JPG7:  u8 = 0xf7;
    pub const JPG8:  u8 = 0xf8;
    pub const JPG9:  u8 = 0xf9;
    pub const JPG10: u8 = 0xfa;
    pub const JPG11: u8 = 0xfb;
    pub const JPG12: u8 = 0xfc;
    pub const JPG13: u8 = 0xfd;
    // Other reserved markers
    pub const COM:   u8 = 0xfe; // Comment
    pub const MRK:   u8 = 0xff; // Marker start
    pub const TEM:   u8 = 0x01; // For temporary private use in arithmetic coding
    pub const ESC:   u8 = 0x00; // Escapes preceeding 0xFF byte (NOT IN SPEC)
    // 0xff02 through 0xffbf are reserved
}

#[derive(Default, Debug)]
struct Frame {
    pub frame_header: FrameHeader,
    pub scans: Vec<Scan>,
    pub lines: Option<NumberOfLines>,
    pub quantization_tables: Vec<QuantizationTable>,
    pub huffman_tables: Vec<HuffmanTable>,
    pub arithmetic_tables: Vec<ArithmeticTable>,
    pub restart_interval: Option<RestartInterval>,
    pub comments: Vec<Comment>,
    pub application_data: Vec<ApplicationData>,
    pub expand_reference: Option<ExpandReference>,
}

#[derive(Default, Debug)]
struct FrameHeader {
    pub marker: u8,                  // SOF or DHP, determines algorithm to decode file
    pub length: u16,                 // Lf
    pub precision: u8,               // P
    pub total_vertical_lines: u16,   // Y
    pub total_horizontal_lines: u16, // X
    pub total_components: u8,        // Nf
    pub components: Vec<FrameComponent>
}

impl FrameHeader {
    fn build(&mut self, marker: &u8, data: &Vec<u8>) {
        if data.len() < 2 {
            // check adds safety for length assignment
            panic!("(FrameHeader::build) (SOF) Not enough byte data"); 
        }
        self.marker = *marker;
        self.length = u16::from_be_bytes([data[0],data[1]]);
        if usize::from(self.length) != data.len() {
            panic!("(FrameHeader::build) (SOF) Byte data length does not correspond to length parameter");
        }
        self.precision = data[2];
        self.total_vertical_lines = u16::from_be_bytes([data[3],data[4]]);
        self.total_horizontal_lines = u16::from_be_bytes([data[5],data[6]]);
        self.total_components = data[7];
        
        let component_length: usize = (self.total_components * 3).into();
        // Each component is 3 bytes
        let component_chunks = data[8..component_length+8].chunks(3);
        for component_bytes in component_chunks.into_iter() {
            let mut component = FrameComponent::default();
            component.build(&component_bytes.to_vec());
            self.components.push(component);
        }


        let component_iter = data.iter().nth(8).into_iter();
        let mut component_data: Vec<u8> = Vec::new();
        for byte in component_iter {
            component_data.push(*byte);
            if component_data.len() == 3 {
                let mut frame_component = FrameComponent::default();
                frame_component.build(&component_data);
                self.components.push(frame_component);
                component_data = Vec::new();
            }
        }
    }
}

#[derive(Default, Debug)]
struct FrameComponent {
    pub bit_stream: u8,                 // Ci
    pub horizontal_sample_factor: u8,   // Hi
    pub vertical_sample_factor: u8,     // Vi
    pub quantization_table_selector: u8 // Tqi
}

impl FrameComponent {
    // Split byte into hi and lo u8 values.
    // Hi: horizontal_sample_factor
    // Lo: vertical_sample_factor
    fn sample_factor(&mut self, byte: &u8) {
        self.horizontal_sample_factor = byte >> 4;
        self.vertical_sample_factor = (byte << 4) >> 4;
    }
    fn build(&mut self, data: &Vec<u8>) {
        if data.len() != 3 {
            panic!("(FrameComponent::build) Byte data not a length of 3.");
        }
        self.bit_stream = data[0];
        self.sample_factor(&data[1]);
        self.quantization_table_selector = data[2];
    }
}

#[derive(Default, Debug)]
struct Scan {
    pub scan_header: ScanHeader,
    // entropy coded segments are separated by RST markers whose intervals are defined by DRI
    //pub entropy_coded_segments: Vec<Vec<u8>>, // ECSi
    pub entropy_coded_segments: Vec<u8> // ECSi
}

#[derive(Default, Debug)]
struct ScanHeader {
    pub length: u16,                     // Ls
    pub total_components: u8,            // Ns
    pub spectral_selection_start: u8,    // Ss
    pub spectral_selection_end: u8,      // Se
    pub successive_approximation_hi: u8, // Ah
    pub successive_approximation_lo: u8, // Al
    pub components: Vec<ScanComponent>
}

impl ScanHeader {
    // Split byte into hi and lo u8 values.
    // Hi: horizontal_sample_factor
    // Lo: vertical_sample_factor
    fn successive_approximation(&mut self, byte: &u8) {
        self.successive_approximation_hi = byte >> 4;
        self.successive_approximation_lo = (byte << 4) >> 4;
    }

    fn build(&mut self, data: &Vec<u8>) {
        if data.len() < 3 {
            panic!("(ScanHeader::build) (SOS) Not enough byte data.");
        }
        self.length = u16::from_be_bytes([data[0],data[1]]);
        if usize::from(self.length) != data.len() {
            panic!("(ScanHeader::build) (SOS) Byte data length does not correspond to length parameter");
        }
        self.total_components = data[2];
        // Ensure the length matches the total_components
        // Each component is 2 bytes and there are 6 bytes of parameters.
        let component_length: usize = (self.total_components * 2).into();
        if component_length + 6 != self.length.into() {
            panic!("(ScanHeader::build) (SOS) Total components parameter does not correspond to length parameter. Component Byte Length: {} | Length: {}", component_length + 6, self.length);
        }
        // Each component is 2 bytes
        let component_chunks = data[3..component_length+3].chunks(2);
        for component_bytes in component_chunks.into_iter() {
            let mut component = ScanComponent::default();
            component.build(&component_bytes.to_vec());
            self.components.push(component);
        }
        self.spectral_selection_start = data[4 + component_length];
        self.spectral_selection_end = data[5 + component_length];
        self.successive_approximation(&data[5 + component_length]);
    }
}

#[derive(Default, Debug)]
struct ScanComponent {
    pub component_selector: u8,    // Cs
    pub dc_entropy_table_dest: u8, // Tdi
    pub ac_entropy_table_dest: u8  // Tai
}

impl ScanComponent {
    fn entropy_table_dest(&mut self, byte: &u8) {
        self.dc_entropy_table_dest = byte >> 4;
        self.ac_entropy_table_dest = (byte << 4) >> 4;
    }
    fn build(&mut self, data: &Vec<u8>) {
        if data.len() != 2 {
            panic!("(ScanComponent::build) Byte data not a length of 2.");
        }
        self.component_selector = data[0];
        self.entropy_table_dest(&data[1]);
    }
}

#[derive(Default, Debug)]
struct QuantizationTable {
    pub length: u16,        // Lq
    pub precision: u8,      // Pq
    pub destination_id: u8, // Tq
    pub element: Vec<u8>    // Qi; limit 64 capacity
}

impl QuantizationTable {
    fn precision_and_destination_id(&mut self, byte: &u8) {
        self.precision = byte >> 4;
        self.destination_id = (byte << 4) >> 4;
    }

    fn build(&mut self, data: &Vec<u8>) {
        if data.len() < 2 {
        // check adds safety for length assignment
            panic!("(QuantizationTable::build) (DQT) Not enough byte data"); 
        }
        self.length = u16::from_be_bytes([data[0],data[1]]);
        if usize::from(self.length) != data.len() {
            panic!("(QuantizationTable::build) (DQT) Byte data length does not correspond to length parameter");
        }
        self.precision_and_destination_id(&data[2]);
        self.element = data[3..].to_vec();
    }
}

#[derive(Default, Debug)]
struct HuffmanEntry {
    pub code: u16,
    pub size: u8
}

#[derive(Default, Debug)]
struct HuffmanTable {
    pub length: u16,                   // Lh
    pub class: u8,                     // Tc
    pub destination_id: u8,            // Th
    pub huffman_size_lengths: Vec<u8>, // Li; 1 >= i <= 16
    pub huffman_values: Vec<u8>,       //Vij; HUFFVAL; 0 >= j <= 255 
    pub table: std::collections::HashMap<u8, HuffmanEntry> // u8 == huffman_value
}

impl HuffmanTable {
    fn class_and_destination_id(&mut self, byte: &u8) {
        self.class = byte >> 4;
        self.destination_id = (byte << 4) >> 4;
    }

    // The output table is referred to as HUFFSIZE in the spec
    fn generate_size_table(& self) -> Vec<u8> {
        let mut huffman_sizes: Vec<u8> = Vec::new();
        for (size, size_len) in self.huffman_size_lengths.iter().enumerate() {
            for _ in 0..*size_len {
                huffman_sizes.push((size + 1).try_into().unwrap());
            }
        }
        return huffman_sizes
    }

    // The output table is referred to as HUFFCODE in the spec
    fn generate_code_table(& self, huffman_sizes: &Vec<u8>) -> Vec<u16> {
        let mut huffman_codes: Vec<u16> = Vec::new();
        let mut code: u16 = 0;
        let mut prev_size: u8 = *huffman_sizes.first().unwrap();
        for size in huffman_sizes.iter() {
            while size != &prev_size {
                code = code << 1;
                prev_size += 1;
            }
            huffman_codes.push(code);
            code += 1;
        }
        return huffman_codes
    }

    fn build_table(&mut self, huffman_sizes: &Vec<u8>, huffman_codes: &Vec<u16>) {
        // Maps each symbol (aka a value) to a huffman size and code
        for (idx, symbol) in self.huffman_values.iter().enumerate() {
            self.table.insert(*symbol, HuffmanEntry {
                code: huffman_codes[idx],
                size: huffman_sizes[idx]
            });
        }
    }

    fn build(&mut self, data: &Vec<u8>) {
        if data.len() < 2 {
            // check adds safety for length assignment
            panic!("(HuffmanTable::build) (DHT) Not enough byte data"); 
        }
        self.length = u16::from_be_bytes([data[0],data[1]]);
        if usize::from(self.length) != data.len() {
            panic!("(HuffmanTable::build) (DHT) Byte data length does not correspond to length parameter");
        }
        self.class_and_destination_id(&data[2]);
        
        // Put all huffman size lengths into huffman_size_lengths vector
        self.huffman_size_lengths = data[3..19].to_vec();

        // Put all huffman values into huffman_values vector
        self.huffman_values = data[19..].to_vec();

        // Decode the huffman table
        let sizes: Vec<u8> =  self.generate_size_table();
        let codes: Vec<u16> = self.generate_code_table(&sizes);
        self.build_table(&sizes, &codes);
    }
}

#[derive(Default, Debug)]
struct ArithmeticTable {
    pub length: u16,        // La
    pub class: u8,          // Tc
    pub destination_id: u8, // Tb
    pub value: u8           // Cs
}

impl ArithmeticTable {
    fn class_and_destination_id(&mut self, byte: &u8) {
        self.class = byte >> 4;
        self.destination_id = (byte << 4) >> 4;
    }

    fn build (&mut self, data: &Vec<u8>) {
        if data.len() < 2 {
            // check adds safety for length assignment
            panic!("(ArithmeticTable::build) (DAC) Not enough byte data"); 
        }
        self.length = u16::from_be_bytes([data[0],data[1]]);
        if usize::from(self.length) != data.len() {
            panic!("(ArithmeticTable::build) (DAC) Byte data length does not correspond to length parameter");
        }
        self.class_and_destination_id(&data[2]);
        self.value = data[3];
    }
}

#[derive(Default, Debug)]
struct RestartInterval {
    pub length: u16,          // Lr
    pub restart_interval: u16 // Ri
}

impl RestartInterval {
    fn build(&mut self, data: &Vec<u8>) {
        if data.len() != 4 {
            // check adds safety for length assignment
            panic!("(RestartInterval::build) (DRI) Byte data not a length of 4"); 
        }
        self.length = u16::from_be_bytes([data[0],data[1]]);
        self.restart_interval = u16::from_be_bytes([data[2],data[3]]);
    }
}

#[derive(Default, Debug)]
struct Comment {
    pub length: u16,           // Lc
    pub comment_bytes: Vec<u8> // Cmi
}

impl Comment {
    fn build(&mut self, data: &Vec<u8>) {
        if data.len() < 2 {
            // check adds safety for length assignment
            panic!("(Comment::build) (COM) Not enough byte data"); 
        }
        self.length = u16::from_be_bytes([data[0],data[1]]);
        if usize::from(self.length) != data.len() {
            panic!("(Comment::build) (COM) Byte data length does not correspond to length parameter");
        }
        self.comment_bytes = data[2..].to_vec();
    }
}

#[derive(Default, Debug)]
struct ApplicationData {
    pub marker: u8,
    pub length: u16,              // Lp
    pub application_data: Vec<u8> // Api
}

impl ApplicationData {
    fn build(&mut self, marker: &u8, data: &Vec<u8>) {
        if data.len() < 2 {
            // check adds safety for length assignment
            panic!("(ApplicationData::build) (APP) Not enough byte data"); 
        }
        self.length = u16::from_be_bytes([data[0],data[1]]);
        if usize::from(self.length) != data.len() {
            panic!("(ApplicationData::build) (APP) Byte data length does not correspond to length parameter");
        }
        self.marker = *marker;
        self.application_data = data[2..].to_vec();
    }
}

#[derive(Default, Debug)]
struct NumberOfLines {
    pub length: u16,     // Ld
    pub total_lines: u16 // NL
}

impl NumberOfLines {
    fn build(&mut self, data: &Vec<u8>) {
        if data.len() != 4 {
            // check adds safety for length assignment
            panic!("(NumberOfLines::build) (DNL) Byte data not a length of 4"); 
        }
        self.length = u16::from_be_bytes([data[0],data[1]]);
        self.total_lines = u16::from_be_bytes([data[2],data[3]]);
    }
}

#[derive(Default, Debug)]
struct ExpandReference {
    pub length: u16,             // Le
    pub expand_horizontally: u8, // Eh
    pub expand_vertically: u8    // Ev
}

impl ExpandReference {
    fn expand_horizontally_and_vertically(&mut self, byte: &u8) {
        self.expand_horizontally = byte >> 4;
        self.expand_vertically = (byte << 4) >> 4;
    }
    fn build(&mut self, data: &Vec<u8>) {
        if data.len() < 2 {
            // check adds safety for length assignment
            panic!("(ExpandReference::build) (EXP) Not enough byte data"); 
        }
        self.length = u16::from_be_bytes([data[0],data[1]]);
        if usize::from(self.length) != data.len() {
            panic!("(ExpandReference::build) (EXP) Byte data length does not correspond to length parameter");
        }
        self.expand_horizontally_and_vertically(&data[2]);
    }
}

#[derive(Debug)]
enum ReadStage {
    Marker,
    Length,
    Segment,
    Scan
}

fn main() {
    let path = "./src/images/rainbow.jpg";
    match std::fs::read(path) {
        Err(x) => println!("path not found: {}", x),
        Ok(bytes) => {
            println!("Scanning in image...");
            let mut stage = ReadStage::Marker;
            let mut current_marker_bytes: [Option<u8>; 2] = [None;2]; // Identify segment to construct based on marker
            let mut segment_length_bytes: [Option<u8>; 2] = [None;2]; // Used for bounds checking
            let mut segment_length: u16 = 0;
            let mut segment_data: Vec<u8> = Vec::new(); // Used to build any segment struct
            //let mut scan_data: Vec<u8> = Vec::new(); // Buffer for bytes directly after scan header
            let mut frame = Frame::default();
            for byte in bytes.iter() {
                // This iterates through all file bytes only once. As it goes, 
                // segment structs are created to represent the entire file in 
                // memory.
                // The following states are considered as we interate:
                // - Reading Marker: If either byte in current_marker
                //   equals 0x00, then we are still reading the marker.
                //   - Knowing the marker is useful for creating the correct
                //   struct with the collected segment_data.
                // - Reading Segment Length: If segment_length equals u16 max
                //   value, then we are still reading the segment length. 
                //   Once the segment_length is set to anything else, we move
                //   onto the next state.
                //   - Knowing the segment length allows us to detect and
                //   ignore cases where 0xFF is not meant to indicate a
                //   marker start.
                // - Reading Segment Data: If segment_length is not equal to
                //   segment_data.len(), then we are still reading bytes into 
                //   segment data. Reading in bytes into an array is useful as
                //   a consistent input parameter for each segment struct's
                //   build function.
                //
                // Each segment and its purpose within the JPEG is defined in
                // the spec: https://www.w3.org/Graphics/JPEG/itu-t81.pdf
                //println!("(Stage) {:?}", stage);
                //println!("Current byte: {:02x?}", *byte);
                match stage {
                    ReadStage::Marker => {
                        if current_marker_bytes[0].is_none() {
                            current_marker_bytes[0] = Some(*byte);
                        } 
                        else if current_marker_bytes[1].is_none() {
                            current_marker_bytes[1] = Some(*byte);
                        }

                        if current_marker_bytes[0] != Some(0xff) {
                            panic!("(ReadStage::Marker) Invalid marker. Expected MSB (big-endian) to equal 0xff. Got {:02x?} instead.", current_marker_bytes[0].unwrap());
                        }
                        else if current_marker_bytes[1].is_some() {
                            if (current_marker_bytes[1] > Some(Markers::TEM) 
                                && current_marker_bytes[1] < Some(Markers::SOF0))
                                || current_marker_bytes[1] == Some(Markers::MRK) {
                                panic!("(ReadStage::Marker) Unknown marker.");
                            }
                            match current_marker_bytes[1] {
                                Some(Markers::TEM)
                                | Some(Markers::SOI)
                                | Some(Markers::EOI) => {
                                    current_marker_bytes = [None;2];
                                    stage = ReadStage::Marker
                                },
                                Some(Markers::ESC) => {
                                    // Include this data into the image data, the 
                                    // 0xff value is escaped by the following 0x00 
                                    // value.
                                    let current_scan = frame.scans.last_mut().unwrap();
                                    current_scan.entropy_coded_segments.push(Markers::MRK);
                                    stage = ReadStage::Scan;
                                },
                                _ => stage = ReadStage::Length,
                            }
                        }
                    },
                    ReadStage::Length => {
                        // Length parameter factors its own 2 byte length into
                        // the total which is why we push to the segment data here
                        segment_data.push(*byte);
                        if segment_length_bytes[0].is_none() {
                            segment_length_bytes[0] = Some(*byte);
                        } 
                        else if segment_length_bytes[1].is_none() {
                            segment_length_bytes[1] = Some(*byte);
                            segment_length = u16::from_be_bytes([
                                segment_length_bytes[0].unwrap(),
                                segment_length_bytes[1].unwrap()
                            ]);
                            stage = ReadStage::Segment;
                        }
                    },
                    ReadStage::Segment => {
                        //println!("Segment Data Length: {}", segment_data.len());
                        //println!("Segment Length: {}", segment_length);
                        
                        segment_data.push(*byte);
                        if segment_data.len() == segment_length.into() {
                            // Data collection has finished
                            // Build with the collected data
                            if (current_marker_bytes[1] >= Some(Markers::SOF0) 
                            && current_marker_bytes[1] <= Some(Markers::SOF3))
                            || (current_marker_bytes[1] >= Some(Markers::SOF5) 
                            && current_marker_bytes[1] <= Some(Markers::SOF7))
                            || (current_marker_bytes[1] >= Some(Markers::SOF9) 
                            && current_marker_bytes[1] <= Some(Markers::SOF11))
                            || (current_marker_bytes[1] >= Some(Markers::SOF13) 
                            && current_marker_bytes[1] <= Some(Markers::SOF15))
                            || current_marker_bytes[1] == Some(Markers::DHP) {
                                frame.frame_header.build(&current_marker_bytes[1].unwrap(), &segment_data);
                            }
                            else if current_marker_bytes[1] == Some(Markers::SOS) {
                                let mut scan = Scan::default();
                                scan.scan_header.build(&segment_data);
                                frame.scans.push(scan);
                                // TODO: Figure out how to add scanned image data to scan
                            }
                            else if current_marker_bytes[1] == Some(Markers::DQT) {
                                frame.quantization_tables.push(QuantizationTable::default());
                                frame.quantization_tables.last_mut().unwrap().build(&segment_data);
                            }
                            else if current_marker_bytes[1] == Some(Markers::DHT) {
                                frame.huffman_tables.push(HuffmanTable::default());
                                frame.huffman_tables.last_mut().unwrap().build(&segment_data);
                            }
                            else if current_marker_bytes[1] == Some(Markers::EXP) {
                                let mut exp = ExpandReference::default();
                                exp.build(&segment_data);
                                frame.expand_reference = Some(exp);
                            }
                            else if current_marker_bytes[1] == Some(Markers::DAC) {
                                frame.arithmetic_tables.push(ArithmeticTable::default());
                                frame.arithmetic_tables.last_mut().unwrap().build(&segment_data);
                            }
                            else if current_marker_bytes[1] == Some(Markers::DNL) {
                                let mut number_of_lines = NumberOfLines::default();
                                number_of_lines.build(&segment_data);
                                frame.lines = Some(number_of_lines);
                            }
                            else if current_marker_bytes[1] == Some(Markers::DRI) {
                                let mut restart_interval = RestartInterval::default();
                                restart_interval.build(&segment_data);
                                frame.restart_interval = Some(restart_interval);
                            }
                            else if current_marker_bytes[1] == Some(Markers::COM) {
                                frame.comments.push(Comment::default());
                                frame.comments.last_mut().unwrap().build(&segment_data);
                            }
                            else if current_marker_bytes[1] >= Some(Markers::APP0) 
                            && current_marker_bytes[1] <= Some(Markers::APP15) {
                                let mut app_data = ApplicationData::default();
                                app_data.build(&current_marker_bytes[1].unwrap(), &segment_data);
                                frame.application_data.push(app_data);
                            }

                            // Restart the process
                            segment_length_bytes = [None;2];
                            segment_length = 0;
                            segment_data = Vec::new();
                            if current_marker_bytes[1] == Some(Markers::SOS) {
                                // Special case where the segment leads into
                                // image data instead of marker data.
                                stage = ReadStage::Scan;
                            }
                            else {
                                stage = ReadStage::Marker;
                            }
                            current_marker_bytes = [None;2];
                        }
                    },
                    ReadStage::Scan => {
                        // Image data in a compressed JPEG is not defined by
                        // a length. We must test any 0xFF value we find as it
                        // can be a marker.
                        if *byte == 0xff {
                            // We may have found a marker
                            current_marker_bytes = [Some(0xff), None];
                            stage = ReadStage::Marker;
                        }
                        else {
                            frame.scans.last_mut().unwrap().entropy_coded_segments.push(*byte);
                        }
                    }
                }
            }
            for table in frame.huffman_tables.iter_mut() {
                println!("{:#?}", table);
            }
            //println!("{:#?}", frame);
        }
    }

    // Decode
    // First, decode the zig-zag sequence of quantized DCT coefficients.
    
    // Need a ScanComponent's dc_entropy_table_dest value and a HuffmanTable 
    // with a class == 0 and destination_id == dc_entropy_table_dest
}

