use std::fs::File;
use std::io::Write;

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
    pub dc_huffman_tables: Vec<HuffmanTable>,
    pub ac_huffman_tables: Vec<HuffmanTable>,
    pub arithmetic_tables: Vec<ArithmeticTable>,
    pub restart_interval: Option<RestartInterval>,
    pub comments: Vec<Comment>,
    pub application_data: Vec<ApplicationData>,
    pub expand_reference: Option<ExpandReference>,
}

#[derive(Default, Debug)]
struct FrameHeader {
    pub marker: u8,                     // SOF or DHP, determines algorithm to decode file
    pub length: u16,                    // Lf
    pub precision: u8,                  // P
    pub total_vertical_lines: u16,      // Y
    pub total_horizontal_lines: u16,    // X
    pub total_components: u8,           // Nf
    pub components: Vec<FrameComponent>
}

impl FrameHeader {
    fn build(&mut self, length: &u16, marker: &u8, data: &Vec<u8>) {
        self.marker = *marker;
        self.length = *length;
        if usize::from(*length) != data.len() {
            panic!("(FrameHeader::build) (SOF) Byte data length does not correspond to length parameter");
        }
        self.precision = data[0];
        self.total_vertical_lines = u16::from_be_bytes([data[1],data[2]]);
        self.total_horizontal_lines = u16::from_be_bytes([data[3],data[4]]);
        self.total_components = data[5];
        
        let component_length: usize = (self.total_components * 3).into();
        // Each component is 3 bytes
        let component_chunks = data[6..component_length+6].chunks(3);
        for component_bytes in component_chunks.into_iter() {
            let mut component = FrameComponent::default();
            component.build(&component_bytes.to_vec());
            self.components.push(component);
        }

        let component_iter = data.iter().nth(6).into_iter();
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
    pub id: u8,                         // Ci
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
        self.id = data[0];
        self.sample_factor(&data[1]);
        self.quantization_table_selector = data[2];
    }
}

#[derive(Default, Debug)]
struct Scan {
    pub scan_header: ScanHeader,
    // entropy coded segments are separated by RST markers whose intervals are defined by DRI
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
    pub components: Vec<ScanComponent>,
}

impl ScanHeader {
    // Split byte into hi and lo u8 values.
    // Hi: horizontal_sample_factor
    // Lo: vertical_sample_factor
    fn successive_approximation(&mut self, byte: &u8) {
        self.successive_approximation_hi = byte >> 4;
        self.successive_approximation_lo = (byte << 4) >> 4;
    }

    fn build(&mut self, length: &u16, data: &Vec<u8>) {
        self.length = *length;
        if usize::from(*length) != data.len() {
            panic!("(ScanHeader::build) (SOS) Byte data length does not correspond to length parameter");
        }
        self.total_components = data[0];
        // Ensure the length matches the total_components
        // Each component is 2 bytes and there are 6 bytes of parameters.
        let component_length: usize = (self.total_components * 2).into();
        if component_length + 4 != usize::from(*length) {
            panic!("(ScanHeader::build) (SOS) Total components parameter does not correspond to length parameter. Component Byte Length: {} | Length: {}", component_length + 4, *length);
        }
        // Each component is 2 bytes
        let component_chunks = data[1..component_length+1].chunks(2);
        for component_bytes in component_chunks.into_iter() {
            let mut component = ScanComponent::default();
            component.build(&component_bytes.to_vec());
            self.components.push(component);
        }
        self.spectral_selection_start = data[2 + component_length];
        self.spectral_selection_end = data[3 + component_length];
        self.successive_approximation(&data[3 + component_length]);
    }
}

#[derive(Default, Debug)]
struct ScanComponent {
    pub id: u8,                    // Cs
    pub dc_entropy_table_dest: u8, // Tdi
    pub ac_entropy_table_dest: u8, // Tai
    prev_dc_coefficient: i16
}

impl ScanComponent {
    fn entropy_table_dest(&mut self, byte: &u8) {
        self.dc_entropy_table_dest = byte >> 4;
        self.ac_entropy_table_dest = byte & 0x0f;
    }
    fn build(&mut self, data: &Vec<u8>) {
        self.id = data[0];
        self.entropy_table_dest(&data[1]);
        self.prev_dc_coefficient = 0;
    }
}

#[derive(Debug)]
struct QuantizationTable {
    pub length: u16,        // Lq
    pub precision: u8,      // Pq
    pub destination_id: u8, // Tq
    pub elements: [u8; 64]  // Qi; limit 64 capacity
}

impl Default for QuantizationTable {
    fn default() -> Self {
        QuantizationTable {
            length: 0,
            precision: 0,
            destination_id: 0,
            elements: [0; 64]
        }
    }
}

impl QuantizationTable {
    fn precision_and_destination_id(&mut self, byte: &u8) {
        self.precision = byte >> 4;
        self.destination_id = (byte << 4) >> 4;
    }

    fn build(&mut self, length: &u16, data: &Vec<u8>) {
        self.length = *length;
        if usize::from(*length) != data.len() {
            panic!("(QuantizationTable::build) (DQT) Byte data length does not correspond to length parameter");
        }
        self.precision_and_destination_id(&data[0]);
        for (idx, element) in data[1..].iter().enumerate() {
            self.elements[idx] = *element;
        }
    }
}

#[derive(Default, Debug)]
struct HuffmanTable {
    pub length: u16,                   // Lh
    pub class: u8,                     // Tc
    pub destination_id: u8,            // Th
    pub huffman_size_lengths: [u8; 16], // Li; 1 >= i <= 16
    pub huffman_values: Vec<u8>,       //Vij; HUFFVAL; 0 >= j <= 255
    // Below are used for decoding procedure
    pub mincode: [u16; 16],
    pub maxcode: [Option<u16>; 16],
    pub valptr: [usize; 16],
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
        huffman_codes.push(code);
        code += 1;
        for size in huffman_sizes[1..].iter() {
            while size != &prev_size {
                code = code << 1;
                prev_size += 1;
            }
            huffman_codes.push(code);
            code += 1;
        }
        return huffman_codes
    }

    fn decoder_tables(&mut self, huffman_codes: &Vec<u16>) {
        let mut j: usize = 0;
        for i in 0..16 {
            if self.huffman_size_lengths[i] == 0 {
                self.maxcode[i] = None;
            }
            else {
                self.valptr[i] = j;
                self.mincode[i] = *huffman_codes.get(j).unwrap();
                j += usize::from(self.huffman_size_lengths[i] - 1);
                self.maxcode[i] = Some(*huffman_codes.get(j).unwrap());
                j += 1;
            }
        }
    }

    fn build(&mut self, length: &u16, data: &Vec<u8>) {
        self.length = *length;
        if usize::from(*length) != data.len() {
            panic!("(HuffmanTable::build) (DHT) Byte data length does not correspond to length parameter");
        }
        self.class_and_destination_id(&data[0]);
        // Put all huffman size lengths into huffman_size_lengths vector
        for (idx, size) in data[1..17].iter().enumerate() {
            self.huffman_size_lengths[idx] = *size;
        }
        // Put all huffman values into huffman_values vector
        self.huffman_values = data[17..].to_vec();
        // Decode the huffman table
        let sizes: Vec<u8> =  self.generate_size_table();
        let codes: Vec<u16> = self.generate_code_table(&sizes);
        self.decoder_tables(&codes);
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

    fn build (&mut self, length: &u16, data: &Vec<u8>) {
        self.length = *length;
        if usize::from(*length) != data.len() {
            panic!("(ArithmeticTable::build) (DAC) Byte data length does not correspond to length parameter");
        }
        self.class_and_destination_id(&data[0]);
        self.value = data[1];
    }
}

#[derive(Default, Debug)]
struct RestartInterval {
    pub length: u16,  // Lr
    pub interval: u16 // Ri
}

impl RestartInterval {
    fn build(&mut self, length: &u16, data: &Vec<u8>) {
        if data.len() != usize::from(*length) {
            // check adds safety for length assignment
            panic!("(RestartInterval::build) (DRI) Byte data not a length of 4"); 
        }
        self.length = *length;
        self.interval = u16::from_be_bytes([data[0],data[1]]);
    }
}

#[derive(Default, Debug)]
struct Comment {
    pub length: u16,           // Lc
    pub comment_bytes: Vec<u8> // Cmi
}

impl Comment {
    fn build(&mut self, length: &u16, data: &Vec<u8>) {
        self.length = *length;
        if usize::from(*length) != data.len() {
            panic!("(Comment::build) (COM) Byte data length does not correspond to length parameter");
        }
        self.comment_bytes = data[0..].to_vec();
    }
}

#[derive(Default, Debug)]
struct ApplicationData {
    pub marker: u8,
    pub length: u16,              // Lp
    pub application_data: Vec<u8> // Api
}

impl ApplicationData {
    fn build(&mut self, marker: &u8, length: &u16, data: &Vec<u8>) {
        self.length = *length;
        if usize::from(*length) != data.len() {
            panic!("(ApplicationData::build) (APP) Byte data length does not correspond to length parameter");
        }
        self.marker = *marker;
        self.application_data = data[0..].to_vec();
    }
}

#[derive(Default, Debug)]
struct NumberOfLines {
    pub length: u16,     // Ld
    pub total_lines: u16 // NL
}

impl NumberOfLines {
    fn build(&mut self, length: &u16, data: &Vec<u8>) {
        self.length = *length;
        self.total_lines = u16::from_be_bytes([data[0],data[1]]);
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
    fn build(&mut self, length: &u16, data: &Vec<u8>) {
        self.length = *length;
        if usize::from(*length) != data.len() {
            panic!("(ExpandReference::build) (EXP) Byte data length does not correspond to length parameter");
        }
        self.expand_horizontally_and_vertically(&data[0]);
    }
}

#[derive(Debug)]
enum ReadStage {
    Marker,
    Length,
    Segment,
    DHTSegment,
    DQTSegment,
    Scan
}

fn main() {
    std::env::set_var("RUST_BACKTRACE", "1");
    let path = "./src/images/lines.jpg";
    match std::fs::read(path) {
        Err(x) => panic!("path not found: {}", x),
        Ok(bytes) => {
            println!("Scanning in {}...", path.to_string());
            let mut stage = ReadStage::Marker;
            let mut current_marker_bytes: [Option<u8>; 2] = [None;2]; // Identify segment to construct based on marker
            let mut segment_length_bytes: [Option<u8>; 2] = [None;2]; // Used for bounds checking
            let mut segment_length: u16 = 0;
            let mut segment_data: Vec<u8> = Vec::new(); // Used to build any segment struct
            let mut frame = Frame::default();
            let mut dht_table_length: u16 = 17;
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
                            if current_marker_bytes[1] > Some(Markers::TEM) 
                            && current_marker_bytes[1] < Some(Markers::SOF0) {
                                panic!("(ReadStage::Marker) Unknown marker.");
                            }
                            match current_marker_bytes[1] {
                                Some(Markers::TEM)
                                | Some(Markers::SOI)
                                | Some(Markers::EOI) => {
                                    current_marker_bytes = [None;2];
                                    stage = ReadStage::Marker
                                },
                                Some(Markers::MRK) => {
                                    // At any point within a JPEG, one 0xff
                                    // may follow another 0xff. The correct
                                    // way to handle this is to treat all
                                    // sequential 0xff values as one.
                                    //
                                    // This is exactly what we're doing here.
                                }
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
                        if segment_length_bytes[0].is_none() {
                            segment_length_bytes[0] = Some(*byte);
                        } 
                        else if segment_length_bytes[1].is_none() {
                            segment_length_bytes[1] = Some(*byte);
                            segment_length = u16::from_be_bytes([
                                segment_length_bytes[0].unwrap(),
                                segment_length_bytes[1].unwrap()
                            ]) - 2;
                            stage = ReadStage::Segment;
                        }
                    },
                    ReadStage::Segment => {
                        segment_data.push(*byte);
                        if current_marker_bytes[1] == Some(Markers::DHT) {
                            stage = ReadStage::DHTSegment;
                        }
                        else if current_marker_bytes[1] == Some(Markers::DQT) {
                            stage = ReadStage::DQTSegment;
                        }
                        else if segment_data.len() == segment_length.into() {
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
                                frame.frame_header.build(&segment_length, &current_marker_bytes[1].unwrap(), &segment_data);
                            }
                            else if current_marker_bytes[1] == Some(Markers::SOS) {
                                let mut scan = Scan::default();
                                scan.scan_header.build(&segment_length, &segment_data);
                                frame.scans.push(scan);
                            }
                            else if current_marker_bytes[1] == Some(Markers::EXP) {
                                let mut exp = ExpandReference::default();
                                exp.build(&segment_length, &segment_data);
                                frame.expand_reference = Some(exp);
                            }
                            else if current_marker_bytes[1] == Some(Markers::DAC) {
                                frame.arithmetic_tables.push(ArithmeticTable::default());
                                frame.arithmetic_tables.last_mut().unwrap().build(&segment_length, &segment_data);
                            }
                            else if current_marker_bytes[1] == Some(Markers::DNL) {
                                let mut number_of_lines = NumberOfLines::default();
                                number_of_lines.build(&segment_length, &segment_data);
                                frame.lines = Some(number_of_lines);
                            }
                            else if current_marker_bytes[1] == Some(Markers::DRI) {
                                let mut restart_interval = RestartInterval::default();
                                restart_interval.build(&segment_length, &segment_data);
                                frame.restart_interval = Some(restart_interval);
                            }
                            else if current_marker_bytes[1] == Some(Markers::COM) {
                                frame.comments.push(Comment::default());
                                frame.comments.last_mut().unwrap().build(&segment_length, &segment_data);
                            }
                            else if current_marker_bytes[1] >= Some(Markers::APP0) 
                            && current_marker_bytes[1] <= Some(Markers::APP15) {
                                let mut app_data = ApplicationData::default();
                                app_data.build(&current_marker_bytes[1].unwrap(), &segment_length, &segment_data);
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
                    },
                    ReadStage::DHTSegment => {
                        segment_data.push(*byte);
                        
                        // We check length 17 because that accounts for the
                        // 1 ID byte
                        // 16 huffman size bytes
                        // = 17
                        if segment_data.len() == 17 {
                            // segment_data now contains the table id and
                            // the total huffman codes per code size.
                            //
                            // This is enough to calculate the length
                            // of this table. (There can be multiple
                            // huffman tables in one DHT segment)
                            dht_table_length += u16::from(segment_data[1..].iter().sum::<u8>());
                        }
                        else if segment_data.len() == dht_table_length.into() {
                            // Prepare to read the next table
                            let mut table = HuffmanTable::default();
                            table.build(&dht_table_length, &segment_data);
                            if table.class == 0 {
                                frame.dc_huffman_tables.push(table);
                            }
                            else {
                                frame.ac_huffman_tables.push(table);
                            }
                            segment_data = Vec::new();
                            segment_length -= dht_table_length;
                            dht_table_length = 17;
                        }
                        if segment_length == 0 {
                            if segment_data.len() > 0 {
                                panic!("(DHTSegment) DHT segment completed with unused segment data."); 
                            }
                            // Restart the process
                            segment_length_bytes = [None;2];
                            stage = ReadStage::Marker;
                            current_marker_bytes = [None;2];
                        }
                    },
                    ReadStage::DQTSegment => {
                        segment_data.push(*byte);
                        
                        // We check length 17 because that accounts for the
                        // 1 ID byte
                        // 64 quantization byte values
                        // = 65
                        if segment_data.len() == 65 {
                            // segment_data now contains the table id and
                            // all quantization table data.
                            frame.quantization_tables.push(QuantizationTable::default());
                            frame.quantization_tables.last_mut().unwrap().build(&65, &segment_data);
                            segment_data = Vec::new();
                            segment_length -= 65;
                        }
                        if segment_length == 0 {
                            // We've read all quantization tables from this segment
                            if segment_data.len() > 0 {
                                panic!("(DQTSegment) DQT segment completed with unused segment data."); 
                            }
                            // Restart the process
                            segment_length_bytes = [None;2];
                            stage = ReadStage::Marker;
                            current_marker_bytes = [None;2];
                        }
                    }
                }
            }
            
            // determine max sampling factors
            let mut max_vertical_factor = 1;
            let mut max_horizontal_factor = 1;
            for component in frame.frame_header.components.iter() {
                if component.vertical_sample_factor > max_vertical_factor {
                    max_vertical_factor = component.vertical_sample_factor;
                }
                if component.horizontal_sample_factor > max_horizontal_factor {
                    max_horizontal_factor = component.horizontal_sample_factor;
                }
            }
            let width = frame.frame_header.total_horizontal_lines;
            let height = frame.frame_header.total_vertical_lines;
            let width_blocks = (width + 7) / 8;
            let height_blocks = (height + 7) / 8;
            let width_blocks_padding: u16 = width_blocks % max_horizontal_factor as u16;
            let height_blocks_padding: u16 = height_blocks % max_vertical_factor as u16;
            let blocks: Vec<[i16; 64]> = 
                decode_huffman_to_blocks(
                    &mut frame, 
                    &width_blocks, 
                    &height_blocks,
                    &width_blocks_padding,
                    &height_blocks_padding,
                    &max_vertical_factor, 
                    &max_horizontal_factor,
                );

            // mcu structure from outer vector to inner array:
            // 1. mcu
            // 2. component
            // 3. blocks
            // 4. samples
            let mut mcus: Vec<Vec<Vec<[i16; 64]>>> = 
                partition_blocks_to_mcus(
                    &blocks, 
                    &width_blocks, 
                    &height_blocks,
                    &width_blocks_padding,
                    &height_blocks_padding,
                    &max_vertical_factor, 
                    &max_horizontal_factor,
                    &frame.frame_header.components
                );
            mcus = dequantize(
                &mcus,
                &frame.frame_header.components,
                &frame.quantization_tables,
                &max_vertical_factor,
                &max_horizontal_factor
            );
            mcus = idct(&mcus);
            mcus = upscale(
                &mcus, 
                &max_vertical_factor, 
                &max_horizontal_factor,
                &frame.frame_header.components
            );
            mcus = upscale(
                &mcus, 
                &max_vertical_factor, 
                &max_horizontal_factor,
                &frame.frame_header.components
            );
            mcus = 
                ycbcr_to_rgb_mcu(
                    &mcus,
                    &width_blocks, 
                    &height_blocks,
                    &width_blocks_padding,
                    &height_blocks_padding,
                    &frame.frame_header.total_components,
                    &((max_vertical_factor * max_horizontal_factor) as usize),
                    &max_vertical_factor, 
                    &max_horizontal_factor
                );
            // Construct the bmp image
            let image_size: usize = 
                width as usize * 
                height as usize * 
                3 + 
                (
                    (width % 4) as usize *
                    height as usize
                );
            let bmp_data = 
                bmp_data_from_mcus(
                    &mcus, 
                    &frame.frame_header.total_components, 
                    &image_size, 
                    &width, 
                    &height, 
                    &width_blocks, 
                    &width_blocks_padding,
                    &max_vertical_factor, 
                    &max_horizontal_factor, 
                    &frame.frame_header.components
                );
            let path = std::path::Path::new("C:/Users/Nick/projects/jpeg-decode/src/images/output.bmp");
            create_bmp(
                &path, 
                &bmp_data, 
                &(width as usize), 
                &(height as usize), 
                &frame.frame_header.total_components
            );
            println!("Bitmap output created at: {}", path.as_os_str().to_str().unwrap());
        }
    }
}

// Allows reading data bit by bit (as opposed to byte by byte)
// Used for huffman decoding
struct BitReader {
    data: Vec<u8>,
    pub byte_idx: usize,
    pub bit_idx: usize
}

impl BitReader {
    fn new(data: &Vec<u8>) -> Self {
        Self {
            data: data.clone(),
            byte_idx: 0,
            bit_idx: 0
        }
    }

    fn next_bit(&mut self) -> Option<u8> {
        if self.byte_idx >= self.data.len() {
            return None
        }
        let bit = (self.data[self.byte_idx] >> (7 - self.bit_idx)) & 1;
        self.bit_idx += 1;
        if self.bit_idx == 8 {
            self.bit_idx = 0;
            self.byte_idx += 1;
        }
        return Some(bit)
    }
    
    // RECIEVE function in the spec (F.2.2.4)
    fn next_bits(&mut self, length: &u8) -> Option<u16> {
        if *length > 16 {
            panic!("(next_bits) Length supplied is greater than 16. Overflow error.");
        }
        let mut bits: u16 = 0;
        for _ in 0..usize::from(*length) {
            let bit = self.next_bit();
            if bit.is_none() {
                return None
            }
            bits = (bits << 1) | u16::from(bit.expect("End of bitstream."));
        }
        return Some(bits)
    }

    fn align(&mut self) {
        // Align the reader to the 0th bit of the next byte.
        // This is used for the beginning of restart intervals.
        if ((self.byte_idx + 1) >= self.data.len())
        || self.bit_idx == 0 {
            return
        }
        self.bit_idx = 0;
        self.byte_idx += 1;
    }
}

fn next_symbol(bit_reader: &mut BitReader, hf: &HuffmanTable) -> Option<u8> {
    let mut code: u16 = bit_reader.next_bit().unwrap().into();
    let mut idx = 0;
    while idx < 16 && (hf.maxcode[idx].is_none() || hf.maxcode[idx].is_some_and(|max| code > max)) {
        let next_bit: u16 = u16::from(bit_reader.next_bit().unwrap());
        code = (code << 1) + next_bit;
        idx += 1;
    }
    if idx >= 16 {
        println!("Couldn't find symbol '{:16b}'", code);
        return None;
    }
    let mut j: usize = hf.valptr[idx];
    j = j + usize::try_from(code).unwrap() - usize::try_from(hf.mincode[idx]).unwrap();
    return Some(*hf.huffman_values.get(j).unwrap())
}

fn decode_block(
    scan_component: &ScanComponent,
    prev_dc: &i16,
    bit_reader: &mut BitReader,
    dc: &HuffmanTable,
    ac: &HuffmanTable,
    zigzag_map: &[usize; 64]
) -> [i16; 64] {
    let mut data_block: [i16; 64] = [0; 64];
    let dc_coeff_length = 
        next_symbol(bit_reader, dc)
            .expect(format!("Could not find symbol in DC huffman table {}.", scan_component.dc_entropy_table_dest).as_str());
    if dc_coeff_length > 11 {
        panic!("(decode_block) DC coefficient cannot have length greater than 11.")
    }
    // Coefficient initially is unsigned
    let dc_coeff_unsigned = 
        bit_reader
            .next_bits(&dc_coeff_length)
            .expect("Invalid DC coefficient");
    // Convert to signed coefficient (refer to table H.2 in the spec)
    let mut dc_coeff: i16 = dc_coeff_unsigned as i16;
    if dc_coeff_length > 0 && dc_coeff < (1 << (dc_coeff_length - 1)) {
        dc_coeff -= (1 << dc_coeff_length) - 1;
    }
    // We add the previous dc value here, refered to as the predictor.
    data_block[0] = dc_coeff + prev_dc;
    let mut ac_counter: usize = 1;
    while ac_counter < 64 {
        let ac_symbol = 
            next_symbol(bit_reader, ac)
                .expect(format!("Could not find symbol in AC huffman table {}.", scan_component.ac_entropy_table_dest).as_str());
        if ac_symbol == 0x00 {
            // 0x00 is a special symbol which tells us to fill the
            // rest of the mcu with zeros
            //
            // We've already initialized mcu with all zeros,
            // so we stop setting any more non-zero values
            // by returning the mcu.
            return data_block
        }
        let mut preceeding_zeros: usize = usize::from(ac_symbol >> 4);
        if ac_symbol == 0xf0 {
            preceeding_zeros = 16;
        }
        if ac_counter + preceeding_zeros >= 64 {
            panic!("(decode_block) Total preceeding zeros exceeds bounds of current data block");
        }
        // We have already initialized the mcu array with zeros, so we
        // "add" zeros to the mcu by simply adding to the ac_counter.
        ac_counter += preceeding_zeros;
        let ac_coeff_length: u8 = ac_symbol & 0x0f;
        if ac_coeff_length > 10 {
            panic!("(decode_block) AC coefficient length cannot exceed 10.");
        }
        else if ac_coeff_length > 0 {
            let ac_coeff_unsigned = 
                bit_reader
                    .next_bits(&ac_coeff_length) 
                    .expect(format!("AC coefficient length exceeds the end of bitstream. Coefficient length: {}", ac_coeff_length).as_str());
            // Convert to signed coefficient (refer to table H.2 in the spec)
            let mut ac_coeff: i16 = ac_coeff_unsigned.try_into().unwrap();
            if ac_coeff < (1 << (ac_coeff_length - 1)) {
                ac_coeff -= (1 << ac_coeff_length) - 1;
            }
            data_block[zigzag_map[ac_counter]] = ac_coeff;
            ac_counter += 1;
        }
    }
    return data_block
}

fn decode_huffman_to_blocks(
    frame: &Frame, 
    width_blocks: &u16,
    height_blocks: &u16,
    padded_width_blocks: &u16,
    padded_height_blocks: &u16,
    max_vertical_factor: &u8,
    max_horizontal_factor: &u8
) -> Vec<[i16; 64]> {
    // The dimensions of a non-interleaved mcu is 8x8 (the same as a data unit)
    // An interleaved mcu can contain one or more data units per component.
    let mut blocks: Vec<[i16; 64]> = Vec::new();
    let total_mcus: u16 = ((width_blocks + padded_width_blocks) / *max_horizontal_factor as u16) * ((height_blocks + padded_height_blocks) / *max_vertical_factor as u16);
    let zigzag: [usize; 64] = [
        0,  1,  8,  16, 9,  2,  3,  10,
        17, 24, 32, 25, 18, 11, 4,  5,
        12, 19, 26, 33, 40, 48, 41, 34,
        27, 20, 13, 6,  7,  14, 21, 28,
        35, 42, 49, 56, 57, 50, 43, 36,
        29, 22, 15, 23, 30, 37, 44, 51,
        58, 59, 52, 45, 38, 31, 39, 46,
        53, 60, 61, 54, 47, 55, 62, 63
    ];
    let mut blocks_per_component: [u16; 4] = [0; 4];
    for (idx, component) in frame.frame_header.components.iter().enumerate() {
        blocks_per_component[idx] = u16::from(component.vertical_sample_factor * component.horizontal_sample_factor);
    }
    for scan in frame.scans.iter() {
        let mut prev_dc: Vec<i16> = Vec::new(); 
        for _ in 0..frame.frame_header.total_components {
            prev_dc.push(0);
        }
        let mut bit_reader = BitReader::new(&scan.entropy_coded_segments);
        let mut mcu_idx = 0;
        while mcu_idx < total_mcus {
            let restart: bool = frame.restart_interval.as_ref().is_some_and(|ri| mcu_idx % ri.interval == 0);
            for sc in scan.scan_header.components.iter() {
                let cid: usize = sc.id as usize - 1;
                if restart {
                    prev_dc[cid] = 0;
                    bit_reader.align();
                }
                for _ in 0..blocks_per_component[cid] {
                    let block = decode_block(
                            sc,
                            &prev_dc[cid],
                            &mut bit_reader,
                            &frame.dc_huffman_tables.get(sc.dc_entropy_table_dest as usize).unwrap(),
                            &frame.ac_huffman_tables.get(sc.ac_entropy_table_dest as usize).unwrap(),
                            &zigzag
                        );
                    prev_dc[cid] = block[0];
                    blocks.push(block);
                }
            }
            mcu_idx += 1;
        }
    }
    return blocks
}

fn partition_blocks_to_mcus(
    blocks: &Vec<[i16; 64]>, 
    width_blocks: &u16, 
    height_blocks: &u16, 
    width_blocks_padding: &u16, 
    height_blocks_padding: &u16,
    max_vertical_factor: &u8, 
    max_horizontal_factor: &u8,
    frame_components: &Vec<FrameComponent>
) -> Vec<Vec<Vec<[i16; 64]>>> {
    let mut mcus: Vec<Vec<Vec<[i16; 64]>>> = Vec::new();
    let mut blocks_idx = 0;
    let mcu_size = max_vertical_factor * max_horizontal_factor;
    let total_mcus: u16 = ((width_blocks + width_blocks_padding) / *max_horizontal_factor as u16) * ((height_blocks + height_blocks_padding) / *max_vertical_factor as u16);
    let mut mcu_idx = 0;
    while mcu_idx < total_mcus {
        let mut mcu: Vec<Vec<[i16; 64]>> = Vec::new();
        for fc in frame_components.iter() {
            let total_component_blocks = fc.horizontal_sample_factor * fc.vertical_sample_factor;
            if total_component_blocks == 0 {
                continue;
            }
            let mut component: Vec<[i16; 64]> = Vec::new();
            // Add placeholder blocks such that all components
            // contain the same number of blocks
            for _ in 0..mcu_size {
                component.push([0; 64]);
            }
            for cb_y in 0..fc.vertical_sample_factor {
                for cb_x in 0..fc.horizontal_sample_factor {
                    // This indexing places blocks into the correct spot within the component
                    let i: usize = (cb_y * max_horizontal_factor + cb_x) as usize;
                    component[i] = blocks[blocks_idx];
                    blocks_idx += 1;
                }
            }
            mcu.push(component);
        }
        mcus.push(mcu);
        mcu_idx += 1;
    }
    return mcus;
}

fn dequantize_block(block: &[i16; 64], qt: &QuantizationTable) -> [i16; 64] {
    let mut dequantized_block: [i16; 64] = [0; 64];
    for idx in 0..64 {
        dequantized_block[idx] = block[idx] * qt.elements[idx] as i16;
    }
    return dequantized_block
}

fn dequantize(
    mcus: &Vec<Vec<Vec<[i16; 64]>>>,
    frame_components: &Vec<FrameComponent>,
    quantization_tables: &Vec<QuantizationTable>,
    max_vertical_factor: &u8,
    max_horizontal_factor: &u8,
) -> Vec<Vec<Vec<[i16; 64]>>> {
    let mut dequantized_mcus: Vec<Vec<Vec<[i16; 64]>>> = Vec::new();
    for mcu in mcus.iter() {
        let mut dequantized_mcu: Vec<Vec<[i16; 64]>> = Vec::new();
        for fc in frame_components.iter() {
            let total_component_blocks = fc.horizontal_sample_factor * fc.vertical_sample_factor;
            if total_component_blocks == 0 {
                continue;
            }
            let mut dequantized_component: Vec<[i16; 64]> = Vec::new();
            for _ in 0..max_horizontal_factor*max_vertical_factor {
                dequantized_component.push([0; 64]);
            }
            let qt: &QuantizationTable = &quantization_tables[fc.quantization_table_selector as usize];
            for cb_y in 0..fc.vertical_sample_factor {
                for cb_x in 0..fc.horizontal_sample_factor {
                    // This indexing places blocks into the correct spot within the component
                    let i: usize = (cb_y * max_horizontal_factor + cb_x) as usize;
                    dequantized_component[i] = dequantize_block(&mcu[fc.id as usize - 1][i], qt);
                }
            }
            dequantized_mcu.push(dequantized_component);
        }
        dequantized_mcus.push(dequantized_mcu);
    }
    return dequantized_mcus;
}

// Inverse Discrete Cosine Transform (aka DCTIII)
fn idct(mcus: &Vec<Vec<Vec<[i16; 64]>>>) -> Vec<Vec<Vec<[i16; 64]>>> {
    let mut shifted_mcus: Vec<Vec<Vec<[i16; 64]>>> = Vec::new();
    for mcu in mcus.iter() {
        let mut shifted_mcu: Vec<Vec<[i16; 64]>> = Vec::new();
        for component in mcu.iter() {
            let mut shifted_component: Vec<[i16; 64]> = Vec::new();
            for block in component.iter() {
                shifted_component.push(idct_block(block));
            }
            shifted_mcu.push(shifted_component);
        }
        shifted_mcus.push(shifted_mcu);
    }
    return shifted_mcus
}

fn idct_block(block: &[i16; 64]) -> [i16; 64] {
    let mut shifted_block: [i16; 64] = [0; 64];
    let inverse_sqrt_two: f64 = 1_f64 / 2_f64.sqrt();
    for y in 0..8 {
        for x in 0..8 {
            let mut sum: f64 = 0.0;
            for v in 0..8 {
                let mut cv: f64 = 1.0;
                if v == 0 {
                    cv = inverse_sqrt_two;
                }
                for u in 0..8 {
                    let mut cu: f64 = 1.0;
                    if u == 0 {
                        cu = inverse_sqrt_two;
                    }
                    sum += cu 
                        * cv 
                        * block[v * 8 + u] as f64
                        * (
                            (2.0 * x as f64 + 1.0)
                            * u as f64
                            * std::f64::consts::PI
                            / 16.0
                        ).cos()
                        * (
                            (2.0 * y as f64 + 1.0)
                            * v as f64
                            * std::f64::consts::PI
                            / 16.0
                        ).cos();
                }
            }
            sum /= 4.0;
            shifted_block[y * 8 + x] = sum.round() as i16;
        }
    }
    return shifted_block
}

// Each sample in the block is converted into a subblock with dimensions of horizontal_scaling_factor by vertical_scaling_factor. 
// These subblocks are then spread across a set of new blocks. The total number of new blocks is govered by
// horizontal_scaling_factor * vertical_scaling_factor.
//
// The result is an upscaled version of a block based on scaling factors.
fn upscale_block(
    block: &[i16; 64], 
    horizontal_scaling_factor: usize, 
    vertical_scaling_factor: usize
) -> Vec<[i16; 64]> {
    let mut upscaled: Vec<[i16; 64]> = Vec::new();
    if horizontal_scaling_factor * vertical_scaling_factor <= 1 {
        // No work needs to be done when both scaling factors equal 1
        upscaled.push(*block);
        return upscaled
    }
    for _ in 0..vertical_scaling_factor * horizontal_scaling_factor {
        upscaled.push([0; 64]);
    }
    for upscaled_y in 0..vertical_scaling_factor {
        // The upscaled index refers to a sample's index across all blocks in the upscaled vector
        let upscaled_y_idx = upscaled_y * 8;
        for upscaled_x in 0..horizontal_scaling_factor {
            let upscaled_x_idx = upscaled_x * 8;
            for y in 0..8 {
                for x in 0..8 {
                    let upscaled_block_idx: usize = upscaled_y * horizontal_scaling_factor + upscaled_x;
                    let upscaled_sample_idx: usize = y * 8 + x;
                    let sample_y = (y + upscaled_y_idx) / vertical_scaling_factor;
                    let sample_x = (x + upscaled_x_idx) / horizontal_scaling_factor;
                    let sample_idx = sample_y * 8 + sample_x;
                    upscaled[upscaled_block_idx][upscaled_sample_idx] = block[sample_idx];
                }
            }
        }
    }
    return upscaled
}

fn upscale(
    mcus: &Vec<Vec<Vec<[i16; 64]>>>, 
    max_vertical_factor: &u8, 
    max_horizontal_factor: &u8,
    frame_components: &Vec<FrameComponent>
) -> Vec<Vec<Vec<[i16; 64]>>> {
    let mut upscaled_mcus: Vec<Vec<Vec<[i16; 64]>>> = Vec::new();
    let mcu_size = max_horizontal_factor * max_vertical_factor;
    for mcu in mcus.iter() {
        let mut upscaled_mcu: Vec<Vec<[i16; 64]>> = Vec::new();
        for fc in frame_components.iter() {
            let total_component_blocks = fc.horizontal_sample_factor * fc.vertical_sample_factor;
            if total_component_blocks == 0 {
                continue;
            }
            let mut upscaled_component: Vec<[i16; 64]> = Vec::new();
            for _ in 0..mcu_size {
                upscaled_component.push([0; 64]);
            }
            // To place the blocks obtained from the upscale_block function in
            // the correct spot inside each upscaled_component, we linearly map
            // each upscaled block based on the sample factors:
            // 
            // vertical | horizontal | b_idx | ub_idx | uc_idx
            // 1          1            0       0        0     (quarter res)
            //                         0       1        1     (stretch all)
            //                         0       2        2
            //                         0       3        3
            // -----------------------------------------------
            // 2          1            0       0        0     (half res)
            //                         0       1        1     (stretch horizontal)
            //                         1       0        2
            //                         1       1        3
            // -----------------------------------------------
            // 1          2            0       0        0     (half res)
            //                         0       1        2     (stretch vertical)
            //                         1       0        1
            //                         1       1        3
            // -----------------------------------------------
            // 2          2            0       0        0     (full res)
            //                         1       0        1     (no stretching)
            //                         2       0        2
            //                         3       0        3
            let x_scale = (*max_horizontal_factor - fc.horizontal_sample_factor) as usize + 1; 
            let y_scale = (*max_vertical_factor - fc.vertical_sample_factor) as usize + 1;
            let mut b_idx = 0;
            for _ in 0..(fc.vertical_sample_factor * fc.horizontal_sample_factor) as usize {
                let upscaled_blocks: Vec<[i16; 64]> = 
                    upscale_block(
                        &mcu[fc.id as usize - 1][b_idx], 
                        x_scale,
                        y_scale
                    );
                for (ub_idx, ub) in upscaled_blocks.iter().enumerate() {
                    let uc_idx: usize = b_idx + ub_idx * fc.vertical_sample_factor as usize;
                    /*println!("component_id: {} | ub_idx: {} | uc_idx: {}", fc.id, ub_idx, uc_idx);
                    for (sample_idx, sample) in ub.iter().enumerate() {
                        if sample_idx % 8 == 0 && sample_idx != 0 {
                            println!();
                        }
                        print!("{},\t", sample);
                    }
                    println!();*/
                    upscaled_component[uc_idx] = *ub;
                }
                b_idx += 1;
            }
            upscaled_mcu.push(upscaled_component);
        }
        upscaled_mcus.push(upscaled_mcu);
    }
    return upscaled_mcus
}

fn ycbcr_to_rgb_mcu(
    mcus: &Vec<Vec<Vec<[i16; 64]>>>, 
    width_blocks: &u16, 
    height_blocks: &u16, 
    width_blocks_padding: &u16, 
    height_blocks_padding: &u16, 
    total_components: &u8, 
    mcu_size: &usize, 
    max_vertical_factor: &u8, 
    max_horizontal_factor: &u8
) -> Vec<Vec<Vec<[i16; 64]>>> {
    let mut rgb_mcus: Vec<Vec<Vec<[i16; 64]>>> = Vec::new();
    let total_mcus: u16 = ((width_blocks + width_blocks_padding) / *max_horizontal_factor as u16) * ((height_blocks + height_blocks_padding) / *max_vertical_factor as u16);
    // Allocate memory for array access on conversion
    for _ in 0..total_mcus {
        let mut fresh_components: Vec<Vec<[i16; 64]>> = Vec::new();
        for _ in 0..*total_components {
            let mut fresh_blocks: Vec<[i16; 64]> = Vec::new();
            for _ in 0..*mcu_size {
                fresh_blocks.push([0; 64]);
            }
            fresh_components.push(fresh_blocks);
        }
        rgb_mcus.push(fresh_components);
    }
    if *total_components == 1 {
        return mcus.clone();
    }
    else if *total_components == 3 {
        for mcu_idx in 0..total_mcus {
            for block_idx in 0..*mcu_size {
                for pixel_idx in 0..64 {
                    let y =  mcus[mcu_idx as usize][0][block_idx][pixel_idx];
                    let cb = mcus[mcu_idx as usize][1][block_idx][pixel_idx];
                    let cr = mcus[mcu_idx as usize][2][block_idx][pixel_idx];
                    rgb_mcus[mcu_idx as usize][0][block_idx][pixel_idx] = ((y as f32 + 1.402 * cr as f32).round() as i16 + 128).max(0).min(255); 
                    rgb_mcus[mcu_idx as usize][1][block_idx][pixel_idx] = ((y as f32 - (0.344 * cb as f32) - (0.714 * cr as f32)).round() as i16 + 128).max(0).min(255); 
                    rgb_mcus[mcu_idx as usize][2][block_idx][pixel_idx] = ((y as f32 + 1.772 * cb as f32).round() as i16 + 128).max(0).min(255); 
                }
            }
        }
    }
    else {
        panic!("Unsupported number of components");
    }
    return rgb_mcus
}

fn bmp_data_from_mcus(
    mcus: &Vec<Vec<Vec<[i16; 64]>>>, 
    total_components: &u8,
    image_size: &usize,
    width: &u16,
    height: &u16,
    width_blocks: &u16, 
    width_blocks_padding: &u16, 
    max_vertical_factor: &u8,
    max_horizontal_factor: &u8,
    frame_components: &Vec<FrameComponent>
) -> Vec<u8> {
    let mut image_data: Vec<u8> = Vec::with_capacity(*image_size);
    if *total_components == 1 {
        for (mcu_idx, mcu) in mcus.iter().enumerate() {
            for component in mcu.iter() {
                for block in component.iter() {
                    for y in 0..8 {
                        for x in 0..8 {
                            image_data[y * mcu_idx + x] = block[y * 8 + x] as u8;
                        }
                    }
                }
            }
        }
    }
    else {
        let mcu_width: usize = (*width_blocks as usize + *width_blocks_padding as usize) / *max_horizontal_factor as usize;
        for y in (0..*height).rev() {
            let mcu_y = y / (8 * max_vertical_factor) as u16;
            let block_y = y / 8;
            let pixel_y = y % 8;
            for x in 0..*width {
                let mcu_x = x / (8 * max_horizontal_factor) as u16;
                let block_x = x / 8;
                let pixel_x = x % 8;
                let mcu_idx: usize = mcu_y as usize * mcu_width + mcu_x as usize;
                let pixel_idx: usize = pixel_y as usize * 8 + pixel_x as usize;
                let mcu_block_y = block_y % *max_vertical_factor as u16;
                let mcu_block_x = block_x % *max_horizontal_factor as u16;
                let mcu_block_idx: usize = (mcu_block_y * *max_horizontal_factor as u16 + mcu_block_x) as usize;
                for component in frame_components.iter().rev() {
                    let byte = mcus[mcu_idx][component.id as usize - 1][mcu_block_idx][pixel_idx] as u8;
                    image_data.push(byte);
                }
            }
            // Account for padding here
            if width % 4 > 0 {
                for _ in 0..width % 4 {
                    image_data.push(0);
                }
            }
        }
    }
    return image_data;
}

fn create_bmp(path: &std::path::Path, image_data: &Vec<u8>, width: &usize, height: &usize, total_components: &u8) {
    let padding = width % 4;
    // For 24 bits per pixel, or 3 color components, we use 3 bytes per pixel
    let image_size: u32 = *width as u32 * *height as u32 * *total_components as u32 + (padding * height) as u32;
    let file_size: u32 = 54 + image_size;
    // Construct bmp header
    // BM (2), file size (4), unused (4), data offset (4)
    let mut header: [u8; 14] = [0x42, 0x4d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00];
    for (idx, byte) in file_size.to_le_bytes().iter().enumerate() {
        header[idx + 2] = *byte;
    }
    // Construct bmp info header
    let mut info_header: [u8; 40] = [0; 40];
    info_header[0] = 0x28; // size of info header
    for (idx, byte) in width.to_le_bytes().iter().enumerate() {
        info_header[idx + 4] = *byte; // width of image
    }
    for (idx, byte) in height.to_le_bytes().iter().enumerate() {
        info_header[idx + 8] = *byte; // height of image
    }
    info_header[12] = 0x01; // number of planes
    match total_components {
        1 => info_header[14] = 0x01, // 1 bit per pixel
        3 => info_header[14] = 0x18, // 24 bits per pixel
        _ => panic!("Unsupported amount of components. 1 component (greyscale) or 3 components (24 bit) are supported.")
    }
    // offset 16 = type of compression (none)
    // offset 20 = compressed image size, but it can be left at 0 since we didnt compress
    for (idx, byte) in image_size.to_le_bytes().iter().enumerate() {
        info_header[idx + 20] = *byte; // compressed image size
    }
    // offset 24 & 28 = x and y pixels per meter. Skippable.
    // offset 32 = colors used. Skippable.
    // offset 36 = Important colors. 0 means all colors are important
    let mut bmp_data: Vec<u8> = Vec::new();
    bmp_data.extend_from_slice(&header);
    bmp_data.extend_from_slice(&info_header);
    bmp_data.extend(image_data);
    let mut bmp = File::create(path).unwrap();
    bmp.write_all(&bmp_data).expect("Failed to write bmp image");
}

