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
    pub marker: u8,                  // SOF or DHP, determines algorithm to decode file
    pub length: u16,                 // Lf
    pub precision: u8,               // P
    pub total_vertical_lines: u16,   // Y
    pub total_horizontal_lines: u16, // X
    pub total_components: u8,        // Nf
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
    pub mcus: Vec<Vec<[i16; 64]>>,
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
        self.mcus = Vec::new();
        self.prev_dc_coefficient = 0;
    }
}

#[derive(Debug)]
struct QuantizationTable {
    pub length: u16,        // Lq
    pub precision: u8,      // Pq
    pub destination_id: u8, // Tq
    pub elements: [u8; 64]    // Qi; limit 64 capacity
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
        //println!("{:16b} ({}) at size {}", code, code, prev_size);
        code += 1;

        for size in huffman_sizes[1..].iter() {
            while size != &prev_size {
                code = code << 1;
                prev_size += 1;
            }
            huffman_codes.push(code);
            //println!("{:16b} ({}) at size {}", code, code, size);
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
    Scan
}

fn main() {
    std::env::set_var("RUST_BACKTRACE", "1");
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
                        //println!("Segment Data Length: {}", segment_data.len());
                        //println!("Segment Length: {}", segment_length);
                        segment_data.push(*byte);
                        if current_marker_bytes[1] == Some(Markers::DHT) {
                            stage = ReadStage::DHTSegment;
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
                                // TODO: Figure out how to add scanned image data to scan
                            }
                            else if current_marker_bytes[1] == Some(Markers::DQT) {
                                frame.quantization_tables.push(QuantizationTable::default());
                                frame.quantization_tables.last_mut().unwrap().build(&segment_length, &segment_data);
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
                            for byte in segment_data.iter() {
                                println!("{}", byte);
                            }
                            dht_table_length += u16::from(segment_data[1..].iter().sum::<u8>());
                        }
                        else if segment_data.len() == dht_table_length.into() {
                            // Prepare to read the next table
                            let mut table = HuffmanTable::default();
                            table.build(&dht_table_length, &segment_data);
                            //println!("{:#?}", table);
                            if table.class == 0 {
                                frame.dc_huffman_tables.push(table);
                            }
                            else {
                                frame.ac_huffman_tables.push(table);
                            }
                            segment_data = Vec::new();
                            println!("Before subtracting segment length: {} | DHT table length: {}", segment_length, dht_table_length); 
                            segment_length -= dht_table_length;
                            println!("Remaining segment length: {}", segment_length); 
                            dht_table_length = 17;
                        }
                        println!("{}, {}", segment_data.len(), segment_length);
                        if segment_length == 0 {
                            if segment_data.len() > 0 {
                                panic!("(DHTSegment) DHT segment completed with unused segment data."); 
                            }
                            // Restart the process
                            segment_length_bytes = [None;2];
                            stage = ReadStage::Marker;
                            current_marker_bytes = [None;2];
                        }
                    }
                }
            }
            /*for huffman_table in frame.dc_huffman_tables.iter_mut() {
                println!("DC Huffman Tables");
                println!("{:#?}", huffman_table);
                for (code, entry) in huffman_table.table.iter() {
                    println!("Code: {:16b} | Size: {} | Symbol: {}", code, entry.size, entry.symbol);
                }
            }
            for huffman_table in frame.ac_huffman_tables.iter_mut() {
                println!("AC Huffman Tables");
                println!("{:#?}", huffman_table);
                for (code, entry) in huffman_table.table.iter() {
                    println!("Code: {:16b} | Size: {} | Symbol: {}", code, entry.size, entry.symbol);
                }
            }*/
            decode_huffman_to_mcus(&mut frame);
            for scan in frame.scans.iter_mut() {
                for scan_component in scan.scan_header.components.iter_mut() {
                    let frame_component: Option<&FrameComponent> = 
                        frame.frame_header.components
                            .iter()
                            .find(|x| x.id == scan_component.id);
                    if let Some(fc) = frame_component {
                        let quantization_table: Option<&QuantizationTable> = 
                            frame.quantization_tables
                                .iter()
                                .find(|x| x.destination_id == fc.quantization_table_selector);
                        if let Some(qt) = quantization_table {
                            println!("MCUs");
                            println!("{:#?}", scan_component.mcus);
                            println!("After Dequantization");
                            scan_component.mcus = dequantize(scan_component, qt);
                            println!("{:#?}", scan_component.mcus);
                            println!("After IDCT");
                            scan_component.mcus = idct_component(&scan_component.mcus);
                            println!("{:#?}", scan_component.mcus);
                        }
                    }
                }
            }
            /*for (i, byte) in frame.scans.first().unwrap().entropy_coded_segments.iter().enumerate() {
                if i % 16 == 0 {
                    println!("");
                }
                print!("{:0x} ", byte);
            }*/
            //println!("{:#?}", frame);
            /*let hf_dc = frame.huffman_tables.first().unwrap();
            let hf_ac = frame.huffman_tables.get(3).unwrap();
            println!("=== DC ===");
            println!("{:#?}", hf_dc);
            println!("=== AC ===");
            println!("{:#?}", hf_ac);
            let quantized: Vec<u8> = entropy_decoder(
                hf_dc,
                hf_ac,
                &frame.scans.first().unwrap().entropy_coded_segments
            );
            let mut counter = 0;
            println!("[");
            for value in quantized.iter() {
                if counter % 8 == 0 {
                    print!("{}\n", value);
                }
                else {
                    print!("{}, ", value);
                }
                counter += 1;
            }
            println!("]");*/
        }
    }

    // Decode
    // First, decode the zig-zag sequence of quantized DCT coefficients.
    
    // Need a ScanComponent's dc_entropy_table_dest value and a HuffmanTable 
    // with a class == 0 and destination_id == dc_entropy_table_dest
}

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

    fn next_bits(&mut self, length: &u8) -> Option<u16> {
        if *length > 16 {
            panic!("(next_bits) Length supplied is greater than 16. Overflow error.");
        }
        let mut bits: u16 = 0;
        for _ in 0..isize::from(*length) {
            let bit = self.next_bit();
            if bit.is_none() {
                return None
            }
            bits = bits << 1 | u16::from(bit.expect("End of bitstream."));
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
        let bit = bit_reader.next_bit().unwrap();
        code = (code << 1) + u16::from(bit);
        idx += 1;
    }
    if idx >= 16 {
        println!("Couldn't find symbol '{:16b}'", code);
        return None;
    }
    let mut j: usize = hf.valptr[idx];
    j = (j + usize::try_from(code).unwrap()) - usize::try_from(hf.mincode[idx]).unwrap();
    return Some(*hf.huffman_values.get(j).unwrap())
}

fn decode_data_block(
    component: &mut ScanComponent,
    bit_reader: &mut BitReader, 
    dc: &HuffmanTable,
    ac: &HuffmanTable
) -> [i16; 64] {
    let mut data_block: [i16; 64] = [0; 64];
    match next_symbol(bit_reader, dc) {
        Some(dc_symbol) => {
            let coeff_length: u8 = dc_symbol;
            //println!("(DC) coefficient length: {}", coeff_length);
            if coeff_length > 11 {
                panic!("(next_entry) DC coefficient cannot have length greater than 11.")
            }
            match bit_reader.next_bits(&coeff_length) {
                Some(coeff_unsigned) => {
                    //println!("(before negative) DC: {}", coeff_unsigned);
                    // convert to signed value
                    // Refer to table H.2 in the spec
                    let mut coeff: i16 = coeff_unsigned.try_into().unwrap();
                    if coeff_length > 0 && coeff < (1 << (coeff_length - 1)) {
                        coeff -= (1 << coeff_length) - 1;
                    }
                    // We add the previous dc value here as the predictor.
                    data_block[0] = coeff + component.prev_dc_coefficient;
                    component.prev_dc_coefficient = data_block[0];
                },
                None => {
                    panic!("Invalid DC coefficient");
                }
            }
        },
        None => panic!("DC: Could not find symbol in huffman table ID {}.", component.dc_entropy_table_dest)
    }
    let mut ac_counter: usize = 1;
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
    
    while ac_counter < 64 {
        match next_symbol(bit_reader, ac) {
            Some(ac_symbol) => {
                //println!("(AC) Counter: {} | Symbol: {}", ac_counter, ac_symbol);
                //let symbol: u8 = ac_entry.symbol;
                /*if coeff_length > 11 {
                    panic!("(next_entry) DC coefficient cannot have length greater than 11.")
                }*/
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
                println!("(AC) Preceeding zeros: {}", preceeding_zeros);
                if ac_symbol == 0xf0 {
                    preceeding_zeros = 16;
                }
                if ac_counter + preceeding_zeros >= 64 {
                    panic!("(decode_mcu_component) Total preceeding zeros exceeds bounds of current mcu");
                }
                // We have already initialized the mcu array with zeros, so we
                // "add" zeros to the mcu by simply adding to the ac_counter.
                ac_counter += preceeding_zeros;
                let coeff_length: u8 = ac_symbol & 0x0f;
                println!("(AC) Coefficient length: {}", coeff_length);
                if coeff_length > 10 {
                    panic!("(decode_mcu_component) AC coefficient length cannot exceed 10.");
                }
                else if coeff_length > 0 {
                    match bit_reader.next_bits(&coeff_length) {
                        Some(coeff_unsigned) => {
                            //println!("(before negative) AC: {}", coeff_unsigned);
                            // convert to signed value
                            // Refer to table H.2 in the spec
                            let mut coeff: i16 = coeff_unsigned.try_into().unwrap();
                            println!("Coefficient: {:16b} < Shifted: {:16b}?", coeff, (1 << (coeff_length - 1)));
                            if coeff < (1 << (coeff_length - 1)) {
                                coeff -= (1 << coeff_length) - 1;
                            }
                            println!("(AC) Coefficient: {}", coeff);
                        
                            data_block[zigzag[ac_counter]] = coeff;
                            ac_counter += 1;
                        },
                        None => {
                            panic!("Invalid AC coefficient in huffman table ID {}", component.ac_entropy_table_dest)
                        }
                    }
                }
            },
            None => {
                println!("AC {}: Could not find symbol in huffman table.", ac_counter);
            }
        }
    }
    return data_block 
}

fn decode_huffman_to_mcus(frame: &mut Frame) {
    // The dimensions of a non-interleaved mcu is 8x8 (the same as a data unit)
    // An interleaved mcu can contain one or more data units per component.
    //
    // We add 7 and then divide 8 to complete any incomplete MCU
    
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
    let mut data_blocks_per_component: [u16; 4] = [0; 4];
    for (idx, component) in frame.frame_header.components.iter().enumerate() {
        data_blocks_per_component[idx] = u16::from(component.vertical_sample_factor * component.horizontal_sample_factor);
    }
    let data_blocks_per_mcu: u16 = data_blocks_per_component.iter().sum();
    // To complete an incomplete data block, we add 7 and then divide 8
    let total_data_blocks_y: u16 = (frame.frame_header.total_vertical_lines + 7) / 8;
    let total_data_blocks_x: u16 = (frame.frame_header.total_horizontal_lines + 7) / 8;
    let total_mcus = 1 + (total_data_blocks_y + total_data_blocks_x) / data_blocks_per_mcu;
    let scan: &mut Scan = frame.scans.first_mut().unwrap();
    let mut bit_reader = BitReader::new(&scan.entropy_coded_segments);
    for mcu_idx in 0..total_mcus {
        let restart: bool = frame.restart_interval.as_ref().is_some_and(|ri| mcu_idx % ri.interval == 0);
        for (idx, scan_component) in scan.scan_header.components.iter_mut().enumerate() {
            let mut mcu: Vec<[i16; 64]> = Vec::new();
            for _ in 0..data_blocks_per_component[idx] {
                if restart {
                    scan_component.prev_dc_coefficient = 0;
                    bit_reader.align(); // Align bit reader to next bit on restart.
                }
                println!("Start MCU for component {}", scan_component.id);
                println!("DC Table: {} | AC Table: {}", scan_component.dc_entropy_table_dest, scan_component.ac_entropy_table_dest);
                mcu.push(
                    decode_data_block(
                        scan_component,
                        &mut bit_reader,
                        &frame.dc_huffman_tables.get(usize::from(scan_component.dc_entropy_table_dest)).unwrap(),
                        &frame.ac_huffman_tables.get(usize::from(scan_component.ac_entropy_table_dest)).unwrap()
                    )
                );
                println!("Byte: {} | Bit: {}", bit_reader.byte_idx, bit_reader.bit_idx);
            }
            scan_component.mcus.push(mcu);
        }
    }
}

fn dequantize(scan_component: &ScanComponent, quantization_table: &QuantizationTable) -> Vec<Vec<[i16; 64]>> {
    let mut dequantized_mcus = Vec::new();
    for mcu in scan_component.mcus.iter() {
        let mut dequantized_mcu: Vec<[i16; 64]> = Vec::new();
        for block in mcu.iter() {
            let mut dequantized_block: [i16; 64] = [0; 64];
            for (value_idx, value) in block.iter().enumerate() {
                dequantized_block[value_idx] = 
                    i16::from(quantization_table.elements[value_idx]) * value;
            }
            dequantized_mcu.push(dequantized_block);
        }
        dequantized_mcus.push(dequantized_mcu);
    }
    dequantized_mcus
}

// Inverse Discrete Cosine Transform (aka DCTIII)
fn idct_component(dequantized_mcus: &Vec<Vec<[i16; 64]>>) -> Vec<Vec<[i16; 64]>> {
    let mut shifted_mcus: Vec<Vec<[i16; 64]>> = Vec::new();
    for dequantized_mcu in dequantized_mcus.iter() {
        let mut shifted_mcu: Vec<[i16; 64]> = Vec::new();
        for block in dequantized_mcu.iter() {
            shifted_mcu.push(idct_block(block));
        }
        shifted_mcus.push(shifted_mcu);
    }
    shifted_mcus
}

fn idct_block(dequantized_block: &[i16; 64]) -> [i16; 64] {
    let mut shifted_block: [i16; 64] = [0; 64];
    let inverse_sqrt_two: f64 = 1_f64 / 2_f64.sqrt();
    for y in 0..8 {
        for x in 0..8 {
            let mut sum: f64 = 0.0;
            for u in 0..8 {
                let mut cu: f64 = 1.0;
                if u == 0 {
                    cu = inverse_sqrt_two;
                }
                for v in 0..8 {
                    let mut cv: f64 = 1.0;
                    if v == 0 {
                        cv = inverse_sqrt_two;
                    }
                    sum += cu 
                        * cv 
                        * dequantized_block[u * 8 + v] as f64
                        * (
                            (2.0 * x as f64 + 1.0)
                            * v as f64
                            * std::f64::consts::PI
                            / 16.0
                        ).cos()
                        * (
                            (2.0 * y as f64 + 1.0)
                            * u as f64
                            * std::f64::consts::PI
                            / 16.0
                        ).cos();
                }
            }
            sum /= 4.0;
            shifted_block[y * 8 + x] = sum.round() as i16;
        }
    }
    shifted_block
}

// TODO: Write the DECODE method
// This will undo the run length encoding and possibly delta encoding.
// F.2.2.3
/*fn entropy_decoder(hf_dc: &HuffmanTable, hf_ac: &HuffmanTable, entropy_coded_segments: &Vec<u8>) -> Vec<u8> {
    let mut quantized: Vec<u8> = Vec::new();
    let mut built_code: u16 = 0;
    let mut built_size: u8 = 1;
    let mut counter: u8 = 0;
    let mut hf: &HuffmanTable = hf_dc;
    // mincode == hf
    for entropy_code in entropy_coded_segments.iter() {
        println!("{:8b}", entropy_code);
        for idx in 0..8 {
            // Add bit from index (big-endian) within the entropy byte.
            built_code += u16::from((entropy_code << idx) >> 7);
            println!("Code: {:16b} ({}) | Size: {}", built_code, built_code, built_size);
            if built_size >= hf.minsize {
                //println!("built_size is larger. built_size: {} | hf.minsize: {}", built_size, hf.minsize);
                hf.table.get(&built_code).map(|entry| {
                    if entry.size == built_size {
                        // This code matches a code in the huffman table.
                        // Add the value to the quantize table.
                        quantized.push(entry.symbol);
                        built_code = 0;
                        built_size = 0;
                        // Swap to dc huffman table for the first sample of
                        // the data unit. Otherwise, use ac huffman table 
                        // for the 63 other values.
                        counter += 1;
                        if counter % 63 == 0 {
                            counter = 1;
                            hf = hf_dc;
                        }
                        else if counter == 1 {
                            hf = hf_ac;
                        }
                    }
                });
                if built_size > hf.maxsize {
                    panic!("Code length cannot exceed {} bits. Code: {:16b}", hf.maxsize, built_code);
                }
            }
            built_code = built_code << 1;
            built_size += 1;
        }
    }
    return quantized
}*/
