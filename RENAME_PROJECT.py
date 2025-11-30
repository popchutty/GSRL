#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
项目重命名脚本
功能：重命名EIDE工程项目，包括相关文件名和内容修改
"""

import os
import sys
import json
import re
import configparser
import yaml
import shutil
from pathlib import Path

class ProjectRenamer:
    def __init__(self, project_root=None):
        """初始化项目重命名器"""
        self.project_root = Path(project_root) if project_root else Path.cwd()
        self.old_name = None
        self.new_name = None
        
    def get_old_project_name(self):
        """从eide.yml读取原工程名"""
        eide_yml_path = self.project_root / ".eide" / "eide.yml"
        if not eide_yml_path.exists():
            raise FileNotFoundError(f"未找到eide.yml文件: {eide_yml_path}")
        
        with open(eide_yml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        
        self.old_name = data.get("name")
        if not self.old_name:
            raise ValueError("eide.yml中未找到name字段")
        
        print(f"检测到原工程名: {self.old_name}")
        return self.old_name
    
    def validate_new_name(self, new_name):
        """验证新工程名的合法性"""
        if not re.match(r'^[a-zA-Z0-9_]+$', new_name):
            return False, "新工程名只能包含字母、数字和下划线"
        
        if new_name == self.old_name:
            return False, "新工程名与原工程名相同，无需重命名"
        
        return True, "验证通过"
    
    def get_new_project_name_interactive(self):
        """交互式获取新工程名"""
        print(f"\n当前工程名: {self.old_name}")
        print("请输入新的工程名（只能包含字母、数字和下划线）：")
        
        while True:
            try:
                new_name = input("新工程名 > ").strip()
                
                if not new_name:
                    print("❌ 工程名不能为空，请重新输入")
                    continue
                
                is_valid, message = self.validate_new_name(new_name)
                
                if is_valid:
                    self.new_name = new_name
                    print(f"✓ 新工程名验证通过: {self.new_name}")
                    return True
                else:
                    print(f"❌ {message}，请重新输入")
                    continue
                    
            except KeyboardInterrupt:
                print("\n\n用户取消操作")
                return False
            except EOFError:
                print("\n\n输入结束")
                return False
    
    def rename_files_and_folders(self):
        """重命名文件和文件夹"""
        print("\n=== 开始重命名文件和文件夹 ===")
        
        # 2.1 重命名.code-workspace文件
        workspace_file = self.project_root / f"{self.old_name}.code-workspace"
        if workspace_file.exists():
            new_workspace_file = self.project_root / f"{self.new_name}.code-workspace"
            workspace_file.rename(new_workspace_file)
            print(f"✓ 重命名工作区文件: {workspace_file.name} -> {new_workspace_file.name}")
        else:
            # 查找任何.code-workspace文件
            workspace_files = list(self.project_root.glob("*.code-workspace"))
            if workspace_files:
                old_workspace = workspace_files[0]
                new_workspace_file = self.project_root / f"{self.new_name}.code-workspace"
                old_workspace.rename(new_workspace_file)
                print(f"✓ 重命名工作区文件: {old_workspace.name} -> {new_workspace_file.name}")
        
        # 2.2 重命名CubeMX_BSP中的.ioc文件
        cubemx_path = self.project_root / "CubeMX_BSP"
        if cubemx_path.exists():
            old_ioc = cubemx_path / f"{self.old_name}.ioc"
            if old_ioc.exists():
                new_ioc = cubemx_path / f"{self.new_name}.ioc"
                old_ioc.rename(new_ioc)
                print(f"✓ 重命名IOC文件: {old_ioc.name} -> {new_ioc.name}")
        
        # 2.3 重命名RTE文件夹（如果存在）
        mdk_path = cubemx_path / "MDK-ARM"
        rte_path = mdk_path / "RTE"
        if rte_path.exists():
            old_rte_folder = rte_path / f"_{self.old_name}"
            if old_rte_folder.exists():
                new_rte_folder = rte_path / f"_{self.new_name}"
                old_rte_folder.rename(new_rte_folder)
                print(f"✓ 重命名RTE文件夹: {old_rte_folder.name} -> {new_rte_folder.name}")
    
    def validate_all_project_names(self):
        """验证待修改文件中的原工程名是否匹配"""
        print("\n=== 开始验证待修改文件中的原工程名 ===")
        
        validation_errors = []
        
        # 验证eide.yml
        try:
            self._validate_eide_yml()
            print("✓ eide.yml验证通过")
        except ValueError as e:
            validation_errors.append(f"eide.yml: {str(e)}")
        
        # 验证CMakeLists.txt
        try:
            self._validate_cmake()
            print("✓ CMakeLists.txt验证通过")
        except ValueError as e:
            validation_errors.append(f"CMakeLists.txt: {str(e)}")
        
        # 验证env.ini
        try:
            self._validate_env_ini()
            print("✓ env.ini验证通过")
        except ValueError as e:
            validation_errors.append(f"env.ini: {str(e)}")
        
        # 验证.ioc文件
        try:
            self._validate_ioc_file()
            print("✓ .ioc文件验证通过")
        except ValueError as e:
            validation_errors.append(f".ioc文件: {str(e)}")
        
        # 验证flash.cfg文件
        try:
            self._validate_flash_cfg()
            print("✓ flash.cfg文件验证通过")
        except ValueError as e:
            validation_errors.append(f"flash.cfg文件: {str(e)}")
        
        if validation_errors:
            error_message = "原工程名验证失败，发现以下错误：\n" + "\n".join([f"  - {error}" for error in validation_errors])
            raise ValueError(error_message)
        
        print("✓ 所有文件中的原工程名验证通过")
    
    def _validate_eide_yml(self):
        """验证eide.yml文件中的原工程名"""
        eide_yml_path = self.project_root / ".eide" / "eide.yml"
        
        with open(eide_yml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        
        # 检查name字段
        if data.get("name") != self.old_name:
            raise ValueError(f"name字段不匹配，预期:{self.old_name}，实际:{data.get('name')}")
        
        # 检查targets下是否有对应的目标名
        if "targets" in data:
            targets = data["targets"]
            if self.old_name not in targets:
                raise ValueError(f"targets中未找到{self.old_name}")
    
    def _validate_cmake(self):
        """验证CMakeLists.txt文件中的原工程名"""
        cmake_path = self.project_root / "CMakeLists.txt"
        if not cmake_path.exists():
            return
        
        with open(cmake_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 检查CMAKE_PROJECT_NAME
        old_cmake_project_name = f"set(CMAKE_PROJECT_NAME {self.old_name})"
        if old_cmake_project_name not in content:
            raise ValueError(f"CMAKE_PROJECT_NAME不匹配，预期:{old_cmake_project_name}")
    
    def _validate_env_ini(self):
        """验证env.ini文件中的原工程名"""
        env_ini_path = self.project_root / ".eide" / "env.ini"
        if not env_ini_path.exists():
            return
        
        with open(env_ini_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 检查原工程名section
        old_section_header = f"[{self.old_name}]"
        if old_section_header not in content:
            raise ValueError(f"未找到[{self.old_name}]section")
        
        # 检查KEIL_OUTPUT_DIR
        old_keil_output = f"KEIL_OUTPUT_DIR={self.old_name}"
        if old_keil_output not in content:
            raise ValueError(f"KEIL_OUTPUT_DIR值不匹配")
    
    def _validate_ioc_file(self):
        """验证.ioc文件中的原工程名"""
        ioc_file = self.project_root / "CubeMX_BSP" / f"{self.old_name}.ioc"
        # 检查重命名后的文件是否存在，如果已经重命名过，则使用新名称
        if not ioc_file.exists():
            # 可能已经重命名过，跳过验证
            return
        
        with open(ioc_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 检查ProjectManager.ProjectFileName
        old_project_filename = f"ProjectManager.ProjectFileName={self.old_name}.ioc"
        if old_project_filename not in content:
            raise ValueError(f"ProjectManager.ProjectFileName不匹配")
        
        # 检查ProjectManager.ProjectName
        old_project_name = f"ProjectManager.ProjectName={self.old_name}"
        if old_project_name not in content:
            raise ValueError(f"ProjectManager.ProjectName不匹配")
    
    def _validate_flash_cfg(self):
        """验证flash.cfg文件中的原工程名"""
        flash_cfg_path = self.project_root / "flash.cfg"
        if not flash_cfg_path.exists():
            return
        
        with open(flash_cfg_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 检查program行中的.elf文件名
        old_elf_path = f"build/Debug/{self.old_name}.elf"
        if old_elf_path not in content:
            raise ValueError(f"program行中的.elf文件名不匹配，预期包含:{old_elf_path}")
    
    def modify_file_contents(self):
        """修改相关文件内容"""
        print("\n=== 开始修改文件内容 ===")
        
        # 3.1 修改eide.yml
        self._modify_eide_yml()
        
        # 3.2 修改CMakeLists.txt
        self._modify_cmake()
        
        # 3.3 修改env.ini
        self._modify_env_ini()
        
        # 3.4 修改files.options.yml
        self._modify_files_options_yml()
        
        # 3.5 修改.ioc文件
        self._modify_ioc_file()
        
        # 3.6 修改flash.cfg文件
        self._modify_flash_cfg()
    
    def _modify_eide_yml(self):
        """修改eide.yml文件"""
        eide_yml_path = self.project_root / ".eide" / "eide.yml"
        
        # 直接读取文件内容进行字符串替换，避免yaml.dump破坏格式
        with open(eide_yml_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 1. 修改name字段（第二行）
        old_name_line = f"name: {self.old_name}"
        new_name_line = f"name: {self.new_name}"
        content = content.replace(old_name_line, new_name_line)
        
        # 2. 修改targets下的目标名
        # 匹配模式：targets: 后面紧跟的目标名（带有正确的缩进）
        old_target_pattern = f"targets:\n  {self.old_name}:"
        new_target_pattern = f"targets:\n  {self.new_name}:"
        content = content.replace(old_target_pattern, new_target_pattern)
        
        # 3. 修改incList中的RTE路径
        old_rte_path = f"CubeMX_BSP/MDK-ARM/RTE/_{self.old_name}"
        new_rte_path = f"CubeMX_BSP/MDK-ARM/RTE/_{self.new_name}"
        content = content.replace(old_rte_path, new_rte_path)
        
        with open(eide_yml_path, 'w', encoding='utf-8') as f:
            f.write(content)
        
        print("✓ 修改eide.yml完成")
    
    def _modify_cmake(self):
        """修改CMakeLists.txt文件"""
        cmake_path = self.project_root / "CMakeLists.txt"
        if not cmake_path.exists():
            return
        
        with open(cmake_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 修改CMAKE_PROJECT_NAME
        old_cmake_project_name = f"set(CMAKE_PROJECT_NAME {self.old_name})"
        new_cmake_project_name = f"set(CMAKE_PROJECT_NAME {self.new_name})"
        
        content = content.replace(old_cmake_project_name, new_cmake_project_name)
        
        with open(cmake_path, 'w', encoding='utf-8') as f:
            f.write(content)
        
        print("✓ 修改CMakeLists.txt完成")
    
    def _modify_env_ini(self):
        """修改env.ini文件"""
        env_ini_path = self.project_root / ".eide" / "env.ini"
        if not env_ini_path.exists():
            return
        
        # 直接读取文件内容进行字符串替换，避免configparser改变键名大小写
        with open(env_ini_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 替换section名和KEIL_OUTPUT_DIR值
        old_section_header = f"[{self.old_name}]"
        new_section_header = f"[{self.new_name}]"
        old_keil_output = f"KEIL_OUTPUT_DIR={self.old_name}"
        new_keil_output = f"KEIL_OUTPUT_DIR={self.new_name}"
        
        content = content.replace(old_section_header, new_section_header)
        content = content.replace(old_keil_output, new_keil_output)
        
        with open(env_ini_path, 'w', encoding='utf-8') as f:
            f.write(content)
        
        print("✓ 修改env.ini完成")
    
    def _modify_files_options_yml(self):
        """修改files.options.yml文件"""
        options_yml_path = self.project_root / ".eide" / "files.options.yml"
        if not options_yml_path.exists():
            return
        
        # 直接读取文件内容进行字符串替换，避免yaml.dump破坏注释和格式
        with open(options_yml_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 使用正则表达式匹配并替换options下的目标名
        # 匹配模式：options: 后面的目标名（带有正确的缩进）
        old_pattern = f"options:\n    {self.old_name}:"
        new_pattern = f"options:\n    {self.new_name}:"
        
        if old_pattern in content:
            content = content.replace(old_pattern, new_pattern)
        
        with open(options_yml_path, 'w', encoding='utf-8') as f:
            f.write(content)
        
        print("✓ 修改files.options.yml完成")
    
    def _modify_ioc_file(self):
        """修改.ioc文件"""
        ioc_file = self.project_root / "CubeMX_BSP" / f"{self.new_name}.ioc"
        if not ioc_file.exists():
            return
        
        with open(ioc_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 修改ProjectManager.ProjectFileName和ProjectManager.ProjectName
        old_project_filename = f"ProjectManager.ProjectFileName={self.old_name}.ioc"
        new_project_filename = f"ProjectManager.ProjectFileName={self.new_name}.ioc"
        old_project_name = f"ProjectManager.ProjectName={self.old_name}"
        new_project_name = f"ProjectManager.ProjectName={self.new_name}"
        
        content = content.replace(old_project_filename, new_project_filename)
        content = content.replace(old_project_name, new_project_name)
        
        with open(ioc_file, 'w', encoding='utf-8') as f:
            f.write(content)
        
        print("✓ 修改.ioc文件完成")
    
    def _modify_flash_cfg(self):
        """修改flash.cfg文件"""
        flash_cfg_path = self.project_root / "flash.cfg"
        if not flash_cfg_path.exists():
            return
        
        with open(flash_cfg_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 修改program行中的.elf文件名
        old_elf_path = f"build/Debug/{self.old_name}.elf"
        new_elf_path = f"build/Debug/{self.new_name}.elf"
        
        content = content.replace(old_elf_path, new_elf_path)
        
        with open(flash_cfg_path, 'w', encoding='utf-8') as f:
            f.write(content)
        
        print("✓ 修改flash.cfg文件完成")
    
    def rename_project(self, new_name=None):
        """执行完整的项目重命名流程"""
        try:
            print("=== 项目重命名开始 ===")
            
            # 步骤0：读取原工程名
            self.get_old_project_name()
            
            # 步骤1：获取新工程名
            if new_name:
                # 如果提供了参数，使用参数
                is_valid, message = self.validate_new_name(new_name)
                if not is_valid:
                    if "相同" in message:
                        print(message)
                        return
                    else:
                        raise ValueError(message)
                self.new_name = new_name
                print(f"新工程名验证通过: {self.new_name}")
            else:
                # 交互式获取新工程名
                if not self.get_new_project_name_interactive():
                    return
            
            # 步骤2：前置验证所有文件中的原工程名（重要：在任何修改操作之前）
            self.validate_all_project_names()
            
            # 步骤3：重命名文件和文件夹
            self.rename_files_and_folders()
            
            # 步骤4：修改文件内容
            self.modify_file_contents()
            
            print(f"\n=== 项目重命名完成 ===")
            print(f"原工程名: {self.old_name}")
            print(f"新工程名: {self.new_name}")
            print("所有操作已成功完成！")
            
        except Exception as e:
            print(f"\n❌ 重命名过程中出现错误: {str(e)}")
            raise
        finally:
            # 等待用户按键后退出
            print("\n按任意键退出...")
            try:
                input()
            except (KeyboardInterrupt, EOFError):
                pass

def main():
    """主函数"""
    new_project_name = None
    
    # 检查是否提供了命令行参数
    if len(sys.argv) == 2:
        new_project_name = sys.argv[1]
    elif len(sys.argv) > 2:
        print("用法: python RENAME_PROJECT.py [新工程名]")
        print("如果不提供新工程名，将进入交互式输入模式")
        print("新工程名只能包含字母、数字和下划线")
        sys.exit(1)
    
    try:
        renamer = ProjectRenamer()
        renamer.rename_project(new_project_name)
    except Exception:
        print(f"脚本执行失败，重命名操作未进行")
        sys.exit(1)

if __name__ == "__main__":
    main()