#发布
cargo embed
cargo embed --release
#调试模式
cargo embed with_rtt
cargo embed with_rtt --release
#工具
probe-rs-cli list

#bin工具
$ cargo install cargo-binutils
$ rustup component add llvm-tools-preview

objcopy
Transform the output of Cargo (ELF) into binary format.
$ cargo objcopy --release -- -O binary target/app.bin
其它例子:
$ cargo objcopy --target riscv32imac-unknown-none-elf --example ferris --release --features=lcd -- -O binary ferris.bin
或者指定文件转换(必须加--strip-all)
$ rust-objcopy target/thumbv6m-none-eabi/release/stm32g071gb --strip-all -O binary target/app2.bin