#include "../crypto.hpp"
#include "../string.hpp"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <gmpxx.h>

//RSA Keys
std::string publickey="-----BEGIN PUBLIC KEY-----\n"\
"MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAy8Dbv8prpJ/0kKhlGeJY\n"\
"ozo2t60EG8L0561g13R29LvMR5hyvGZlGJpmn65+A4xHXInJYiPuKzrKUnApeLZ+\n"\
"vw1HocOAZtWK0z3r26uA8kQYOKX9Qt/DbCdvsF9wF8gRK0ptx9M6R13NvBxvVQAp\n"\
"fc9jB9nTzphOgM4JiEYvlV8FLhg9yZovMYd6Wwf3aoXK891VQxTr/kQYoq1Yp+68\n"\
"i6T4nNq7NWC+UNVjQHxNQMQMzU6lWCX8zyg3yH88OAQkUXIXKfQ+NkvYQ1cxaMoV\n"\
"PpY72+eVthKzpMeyHkBn7ciumk5qgLTEJAfWZpe4f4eFZj/Rc8Y8Jj2IS5kVPjUy\n"\
"wQIDAQAB\n"\
"-----END PUBLIC KEY-----\n";
std::string privatekey="-----BEGIN RSA PRIVATE KEY-----\n"\
"MIIEowIBAAKCAQEAy8Dbv8prpJ/0kKhlGeJYozo2t60EG8L0561g13R29LvMR5hy\n"\
"vGZlGJpmn65+A4xHXInJYiPuKzrKUnApeLZ+vw1HocOAZtWK0z3r26uA8kQYOKX9\n"\
"Qt/DbCdvsF9wF8gRK0ptx9M6R13NvBxvVQApfc9jB9nTzphOgM4JiEYvlV8FLhg9\n"\
"yZovMYd6Wwf3aoXK891VQxTr/kQYoq1Yp+68i6T4nNq7NWC+UNVjQHxNQMQMzU6l\n"\
"WCX8zyg3yH88OAQkUXIXKfQ+NkvYQ1cxaMoVPpY72+eVthKzpMeyHkBn7ciumk5q\n"\
"gLTEJAfWZpe4f4eFZj/Rc8Y8Jj2IS5kVPjUywQIDAQABAoIBADhg1u1Mv1hAAlX8\n"\
"omz1Gn2f4AAW2aos2cM5UDCNw1SYmj+9SRIkaxjRsE/C4o9sw1oxrg1/z6kajV0e\n"\
"N/t008FdlVKHXAIYWF93JMoVvIpMmT8jft6AN/y3NMpivgt2inmmEJZYNioFJKZG\n"\
"X+/vKYvsVISZm2fw8NfnKvAQK55yu+GRWBZGOeS9K+LbYvOwcrjKhHz66m4bedKd\n"\
"gVAix6NE5iwmjNXktSQlJMCjbtdNXg/xo1/G4kG2p/MO1HLcKfe1N5FgBiXj3Qjl\n"\
"vgvjJZkh1as2KTgaPOBqZaP03738VnYg23ISyvfT/teArVGtxrmFP7939EvJFKpF\n"\
"1wTxuDkCgYEA7t0DR37zt+dEJy+5vm7zSmN97VenwQJFWMiulkHGa0yU3lLasxxu\n"\
"m0oUtndIjenIvSx6t3Y+agK2F3EPbb0AZ5wZ1p1IXs4vktgeQwSSBdqcM8LZFDvZ\n"\
"uPboQnJoRdIkd62XnP5ekIEIBAfOp8v2wFpSfE7nNH2u4CpAXNSF9HsCgYEA2l8D\n"\
"JrDE5m9Kkn+J4l+AdGfeBL1igPF3DnuPoV67BpgiaAgI4h25UJzXiDKKoa706S0D\n"\
"4XB74zOLX11MaGPMIdhlG+SgeQfNoC5lE4ZWXNyESJH1SVgRGT9nBC2vtL6bxCVV\n"\
"WBkTeC5D6c/QXcai6yw6OYyNNdp0uznKURe1xvMCgYBVYYcEjWqMuAvyferFGV+5\n"\
"nWqr5gM+yJMFM2bEqupD/HHSLoeiMm2O8KIKvwSeRYzNohKTdZ7FwgZYxr8fGMoG\n"\
"PxQ1VK9DxCvZL4tRpVaU5Rmknud9hg9DQG6xIbgIDR+f79sb8QjYWmcFGc1SyWOA\n"\
"SkjlykZ2yt4xnqi3BfiD9QKBgGqLgRYXmXp1QoVIBRaWUi55nzHg1XbkWZqPXvz1\n"\
"I3uMLv1jLjJlHk3euKqTPmC05HoApKwSHeA0/gOBmg404xyAYJTDcCidTg6hlF96\n"\
"ZBja3xApZuxqM62F6dV4FQqzFX0WWhWp5n301N33r0qR6FumMKJzmVJ1TA8tmzEF\n"\
"yINRAoGBAJqioYs8rK6eXzA8ywYLjqTLu/yQSLBn/4ta36K8DyCoLNlNxSuox+A5\n"\
"w6z2vEfRVQDq4Hm4vBzjdi3QfYLNkTiTqLcvgWZ+eX44ogXtdTDO7c+GeMKWz4XX\n"\
"uJSUVL5+CVjKLjZEJ6Qc2WZLl94xSwL71E41H4YciVnSCQxVc4Jw\n"\
"-----END RSA PRIVATE KEY-----\n";

//AES Keys
std::string key="01234567890123456789012345678901";
std::string iv="01234567890123456";

//HMAC Key
std::string hmac_key="key";

int main(int argc,char* argv[])
{
	std::string plaintext="The quick brown fox jumps over the lazy dog";
	std::string encryptedtext="";
	std::string decryptedtext="";
	std::string hash="";

	//RSA Testing
	std::cout<<"RSA"<<std::endl;
	encryptedtext=msl::encrypt_rsa(plaintext,publickey);
	decryptedtext=msl::decrypt_rsa(encryptedtext,privatekey);
	std::cout<<msl::to_hex_string(encryptedtext)<<std::endl;
	std::cout<<decryptedtext<<std::endl;
	std::cout<<std::endl;

	//AES Testing
	std::cout<<"AES256"<<std::endl;
	encryptedtext=msl::encrypt_aes256(plaintext,key,iv);
	decryptedtext=msl::decrypt_aes256(encryptedtext,key,iv);
	std::cout<<msl::to_hex_string(encryptedtext)<<std::endl;
	std::cout<<decryptedtext<<std::endl;
	std::cout<<std::endl;

	//Hash Testing
	std::cout<<"MD5"<<std::endl;
	hash=msl::hash_md5(plaintext);
	std::cout<<msl::to_hex_string(hash)<<std::endl;
	std::cout<<std::endl;

	std::cout<<"SHA160"<<std::endl;
	hash=msl::hash_sha160(plaintext);
	std::cout<<msl::to_hex_string(hash)<<std::endl;
	std::cout<<std::endl;

	std::cout<<"SHA256"<<std::endl;
	hash=msl::hash_sha256(plaintext);
	std::cout<<msl::to_hex_string(hash)<<std::endl;
	std::cout<<std::endl;

	std::cout<<"SHA512"<<std::endl;
	hash=msl::hash_sha512(plaintext);
	std::cout<<msl::to_hex_string(hash)<<std::endl;
	std::cout<<std::endl;

	//HMAC Testing
	std::cout<<"HMAC_MD5"<<std::endl;
	hash=msl::hmac_md5(hmac_key,plaintext);
	std::cout<<msl::to_hex_string(hash)<<std::endl;
	std::cout<<std::endl;

	std::cout<<"HMAC_SHA160"<<std::endl;
	hash=msl::hmac_sha160(hmac_key,plaintext);
	std::cout<<msl::to_hex_string(hash)<<std::endl;
	std::cout<<std::endl;

	std::cout<<"HMAC_SHA256"<<std::endl;
	hash=msl::hmac_sha256(hmac_key,plaintext);
	std::cout<<msl::to_hex_string(hash)<<std::endl;
	std::cout<<std::endl;

	std::cout<<"HMAC_SHA512"<<std::endl;
	hash=msl::hmac_sha512(hmac_key,plaintext);
	std::cout<<msl::to_hex_string(hash)<<std::endl;
	std::cout<<std::endl;

	return 0;
}