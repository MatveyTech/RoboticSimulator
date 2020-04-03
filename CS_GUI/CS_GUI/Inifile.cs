using System;
using System.Runtime.InteropServices;
using System.Text;

public static class IniFile
{
    static String inifilePath = @"c:\temp\tes_shared.ini";
        
    public static void WriteBoolean(string Key, bool Value)
    {
        WritePrivateProfileString("General", Key, Value.ToString(), inifilePath);
    }




















    [DllImport("kernel32")]
    private static extern long WritePrivateProfileString(string section, string key, string val, string filePath);
    [DllImport("kernel32")]
    private static extern int GetPrivateProfileString(string section, string key, string def, StringBuilder retVal, int size, string filePath);

    public static void WriteValue(string path, string Section, string Key, string Value)
    {
        WritePrivateProfileString(Section, Key, Value, path);
    }

    public static string ReadValue(string path, string Section, string Key)
    {
        StringBuilder temp = new StringBuilder(255);
        int i = GetPrivateProfileString(Section, Key, "", temp, 255, path);
        return temp.ToString();
    }

}