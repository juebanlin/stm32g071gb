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

#BOOT0注意事项
stm32g0系列默认boot0不受外部boot0 pin控制,使用CubeProgrammer 进入OB-USER Configuration
找到nBOOT_SEL去掉勾选 apply,点击disconnect.或者刷新一次固件即可使用外部boot0引脚控制启动模式
https://blog.csdn.net/zrb2753/article/details/117886821
